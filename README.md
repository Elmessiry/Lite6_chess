Prerequisites 
1. Docker   https://docs.docker.com/desktop/setup/install/windows-install/
2. x server
2.1. install erlang https://www.erlang.org/downloads
2.2. x server   https://sourceforge.net/projects/vcxsrv/  
3. fritz engine 15 or newer
4. dgt board plugins  https://digitalgametechnology.com/support/software/software-downloads
5. chessnut plugins  https://goneill.co.nz/chess.php#chessnut
6. rabbitmq  https://www.rabbitmq.com/docs/install-windows#downloads
7. requirements.txt

For Windows

• in a terminal navigate to local repository, then >pip install -r requirements.txt

• run x server  (mark no access control)

• run docker

• run rabbitmq 

• start a terminal

	1. building the docker image in terminal, Navigate to local repository, then the folder .devcontainer
		> docker build --no-cache --network=host -t my_image .

	2. run docker image command, make sure to update the mounted volume path
		>docker run -it --env DISPLAY=host.docker.internal:0.0 --network="host" --add-host=host.docker.internal:host-gateway -v C:/Users/A/Desktop/ThesisT2/chess:/home/dev_ws/chess --ipc=host --rm --name thisisit my_image

	3. run the rviz ( initiate robot connection)

	 For simulation
		ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py add_vacuum_gripper:=true

	 For Real arm
		ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.175 add_vacuum_gripper:=true


• start a new terminal to the same container using
	>docker exec -it thisisit /bin/bash
navigate to the mounted volume home/dev_ws/chess/robot/src
python3 main.py
choose simulation or real arm

• Go to Rviz, add the visualization by clicking add on the displays list on the left side of the Rviz window, and adding a MarkerArray then changing its topic to /chess_board_visualization then press enter

• start fritz engine
• in a new terminal navigate to repository folder, then  /windows/src
      run  python3 main.py  and choose the fritz window if prompted and the robot color
