import time

import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import click
import numpy as np
from arx5_interface import Arx5CartesianController, EEFState, Gain, LogLevel

"""
This script is to test the solver when the gravity vector is not the default value (pointing down)
To run this script successfully, please hang the robot arm upside down, so the gravity vector becomes [0, 0, 9.81].
"""


def inverse_transformation_matrix(T):
    """Compute the inverse of a 4x4 homogeneous transformation matrix."""
    R_inv = T[:3, :3].T  # Transpose of the rotation matrix
    t_inv = -R_inv @ T[:3, 3]  # Invert the translation
    
    # Construct the inverse transformation matrix
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
@click.option("--urdf_path", "-u", default="../models/arx5_webcam.urdf", help="URDF file path")
def main(model: str, interface: str, urdf_path: str):
    np.set_printoptions(precision=3, suppress=True)

    robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
    robot_config.gravity_vector = np.array([0, 9.81*np.sqrt(2)/2, 9.81*np.sqrt(2)   /2])


    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "cartesian_controller", robot_config.joint_dof
    )


    arx5_controller = Arx5CartesianController(
        robot_config, controller_config, interface, urdf_path
    )

    gain = arx5_controller.get_gain()
    arx5_controller.set_gain(gain)  

    arx5_controller.reset_to_home()







    T_left_shoulder_to_world = np.array([
        [1,  0,     0,       0.18],
        [0, -0.707, 0.707, 0.14563],
        [0, -0.707, -0.707,  0.037426],
        [0,  0,     0,          1]
        ])  
    
    T_world_to_left_shoulder = inverse_transformation_matrix(T_left_shoulder_to_world)


   


    end_pose = np.array([0.4, 0, 0.3, 0, 0, 0]) 



    eefstate = EEFState(end_pose, 0)
    current_state = arx5_controller.get_eef_state()
    eefstate.timestamp = current_state.timestamp + 4
    arx5_controller.set_eef_cmd(eefstate)

    time.sleep(10)


    right_ee_state = arx5_controller.get_eef_state()
    right_shoulder_pose = right_ee_state.pose_6d()


    print(right_shoulder_pose)


    time.sleep(1000)



if __name__ == "__main__":
    main()
