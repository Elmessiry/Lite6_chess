import os
import logging
import logging.config
import yaml
from datetime import datetime

def setup_logging(component=None):
    """
    Setup logging configuration for the entire application
    
    Args:
        component: Optional component name to get a specific logger
        
    Returns:
        Logger instance
    """
    # Get paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    config_path = os.path.join(project_root, 'config', 'logging_config.yaml')
    logs_dir = os.path.join(project_root, 'logs')
    
    # Create logs directory
    os.makedirs(logs_dir, exist_ok=True)
    
    # Create log filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(logs_dir, f"chess_robot_{timestamp}.log")
    
    # Load and update config
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            config['handlers']['file']['filename'] = log_file
        
        # Apply configuration
        logging.config.dictConfig(config)
    except Exception as e:
        # Fallback to basic configuration if loading fails
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[logging.StreamHandler()]
        )
        logging.error(f"Failed to load logging configuration: {e}")
    
    # Return the requested logger
    if component:
        return logging.getLogger(component)
    return logging.getLogger() 