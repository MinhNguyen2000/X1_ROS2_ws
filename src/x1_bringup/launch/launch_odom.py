from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # --- Set required paths
    pkg_path = get_package_share_directory("x1_bringup")
    ekf_params_path = os.path.join(pkg_path, "config", "ekf_params.yaml")

    # --- Define launch arguments

    # --- Nodes and launch descriptions

    # TODO - integrate the LiDAR scan matcher package

    # TODO - implement a covariance filter node to publish wheel and IMU covariance

    # TODO - EKF node

