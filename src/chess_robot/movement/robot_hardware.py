import rclpy
import asyncio
from geometry_msgs.msg import PoseStamped, Quaternion
import shape_msgs.msg
import moveit_msgs.msg
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from xarm_msgs.srv import VacuumGripperCtrl, GetInt16
from enum import Enum
from typing import Dict, Any
import yaml
import os

class MoveResult(Enum):
    SUCCESS = "Success"
    PLANNING_FAILED = "Planning failed"
    EXECUTION_FAILED = "Execution failed"
    GRIPPER_FAILED = "Gripper operation failed"
    INVALID_POSE = "Invalid pose"

class RobotHardware:
    """Handles direct robot control and hardware abstraction"""
    
    def __init__(self, node):
        self.node = node
        self.config = self._load_config()
        self.simulation_mode = self._detect_simulation_mode()
        self._setup_action_client()
        self._setup_gripper()

    def _load_config(self) -> Dict[str, Any]:
        """Load robot configuration"""
        config_path = "/home/dev_ws/chess/config/board_config.yaml"
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config['robot']
        except Exception as e:
            self.node.get_logger().error(f"Failed to load config: {e}")
            raise

    def _detect_simulation_mode(self) -> bool:
        """Prompt user to specify if running in simulation or hardware mode"""
        while True:
            try:
                mode = input("Enter 1 for simulation mode or 2 for hardware mode: ")
                if mode == "1":
                    self.node.get_logger().info("User selected simulation mode")
                    return True
                elif mode == "2":
                    self.node.get_logger().info("User selected hardware mode") 
                    return False
                else:
                    self.node.get_logger().info("Invalid input. Please enter 1 or 2.")
            except Exception as e:
                self.node.get_logger().error(f"Error getting user input: {e}")

    def _setup_action_client(self):
        """Setup MoveGroup action client"""
        self.move_group_client = ActionClient(self.node, MoveGroup, 'move_action')
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Action server not available!")
    
    def _setup_gripper(self):
        """Setup vacuum gripper service clients"""
        if not self.simulation_mode:
            self.vacuum_set_client = self.node.create_client(
                VacuumGripperCtrl, 
                '/ufactory/set_vacuum_gripper'
            )
            self.vacuum_get_client = self.node.create_client(
                GetInt16,
                '/ufactory/get_vacuum_gripper'
            )
            self.node.get_logger().info("Setting up gripper services with ufactory paths")
        else:
            self.vacuum_set_client = None
            self.vacuum_get_client = None

    async def move_to_pose(self, x: float, y: float, z: float) -> MoveResult:
        """Execute movement to given pose"""
        try:
            goal_msg = self._create_move_goal(x, y, z)
            success = await self._execute_movement(goal_msg)
            return MoveResult.SUCCESS if success else MoveResult.EXECUTION_FAILED
        except Exception as e:
            self.node.get_logger().error(f"Move to pose failed: {e}")
            return MoveResult.EXECUTION_FAILED

    def _create_move_goal(self, x: float, y: float, z: float) -> MoveGroup.Goal:
        """Create MoveGroup goal message with config parameters"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "lite6"
        
        # Create pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)

        # Setup constraints
        self._setup_planning_parameters(goal_msg)
        position_constraint = self._create_position_constraint(target_pose)
        goal_msg.request.goal_constraints.append(position_constraint)
        orientation_constraint = self._create_orientation_constraint(target_pose)
        goal_msg.request.goal_constraints[0].orientation_constraints.append(orientation_constraint)
        
        return goal_msg

    def _setup_planning_parameters(self, goal_msg: MoveGroup.Goal):
        """Setup workspace bounds and planning parameters from config"""
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        movement_config = self.config['movement']
        goal_msg.request.allowed_planning_time = movement_config['planning_time']
        goal_msg.request.num_planning_attempts = movement_config['planning_attempts']
        goal_msg.request.max_velocity_scaling_factor = movement_config['max_velocity_scaling']
        goal_msg.request.max_acceleration_scaling_factor = movement_config['max_acceleration_scaling']

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = movement_config['replan_attempts']
        goal_msg.planning_options.replan_delay = movement_config['replan_delay']

    async def _execute_movement(self, goal_msg: MoveGroup.Goal) -> bool:
        """Execute the movement with the given goal message"""
        try:
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=65.0)
            
            if not send_goal_future.done():
                self.node.get_logger().error("Timeout waiting for goal acceptance")
                return False
                
            goal_handle = send_goal_future.result()
            if not goal_handle:
                self.node.get_logger().error("Goal rejected by server")
                return False
                
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=65.0)
            
            if not result_future.done():
                self.node.get_logger().error("Timeout waiting for movement result")
                return False
                
            status = result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                return True
            else:
                self._log_error_code(result_future.result().result.error_code.val)
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Movement execution error: {e}")
            return False

    async def control_gripper(self, enable: bool) -> bool:
        """Control vacuum gripper state"""
        if self.simulation_mode:
            self.node.get_logger().info(f"SIMULATION: {'Enabling' if enable else 'Disabling'} vacuum gripper")
            await asyncio.sleep(0.5)
            return True

        try:
            if not self.vacuum_set_client.service_is_ready():
                self.node.get_logger().error("Vacuum set service not ready")
                return False

            set_request = VacuumGripperCtrl.Request()
            set_request.on = enable
            set_request.wait = False
            set_request.timeout = 2.0
            
            future = self.vacuum_set_client.call_async(set_request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
            
            if not future.done():
                self.node.get_logger().error("Service call timed out")
                return False
                
            set_response = future.result()
            
            if set_response.ret != 0:
                self.node.get_logger().error(f"Gripper control failed with return code: {set_response.ret}")
                return False

            await asyncio.sleep(1.0)  # Wait for physical actuation
            return True
                
        except Exception as e:
            self.node.get_logger().error(f"Gripper control failed: {e}")
            return False

    def _create_position_constraint(self, target_pose: PoseStamped) -> Constraints:
        """Create position constraint for movement"""
        constraint = Constraints()
        constraint.name = "goal"
        
        pos_constraint = moveit_msgs.msg.PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = "link_tcp"
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        sphere = shape_msgs.msg.SolidPrimitive()
        sphere.type = shape_msgs.msg.SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        
        constraint.position_constraints.append(pos_constraint)
        return constraint

    def _create_orientation_constraint(self, target_pose: PoseStamped) -> OrientationConstraint:
        """Create orientation constraint to keep gripper facing downward"""
        constraint = OrientationConstraint()
        constraint.header = target_pose.header
        constraint.orientation = target_pose.pose.orientation
        constraint.link_name = "link_tcp"
        constraint.absolute_x_axis_tolerance = 0.01
        constraint.absolute_y_axis_tolerance = 0.01
        constraint.absolute_z_axis_tolerance = 0.01
        constraint.weight = 1.0
        return constraint

    def _log_error_code(self, error_code: int):
        """Log detailed error information"""
        error_messages = {
            moveit_msgs.msg.MoveItErrorCodes.INVALID_GROUP_NAME: "Invalid group name",
            moveit_msgs.msg.MoveItErrorCodes.PLANNING_FAILED: "Planning failed",
            moveit_msgs.msg.MoveItErrorCodes.INVALID_MOTION_PLAN: "Invalid motion plan",
            moveit_msgs.msg.MoveItErrorCodes.NO_IK_SOLUTION: "No inverse kinematics solution"
        }
        error_msg = error_messages.get(error_code, f"Unknown error code: {error_code}")
        self.node.get_logger().error(f"Movement failed: {error_msg}")
