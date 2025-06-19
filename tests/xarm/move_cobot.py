"""
Date: 16/06/2025
Desc: To test movement of UFactory Lite 6 using ManipulatorRobot class
Author: Reuben Lim
NOTE: Change arm IP before use
"""
import numpy as np
import time
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.configs import XarmRobotConfig # Or your robot's config
from lerobot.common.robot_devices.motors.configs import XarmMotorsBusConfig

if __name__ == '__main__':
    robot_config = XarmRobotConfig( # [cite: 2]
        follower_arms={
            "main": XarmMotorsBusConfig(
                port="192.168.1.224",  # <<-- UPDATE THIS PORT
                motors={
                    "joint_1": [1, "ufactory-lite6"], # +/- 360 degrees
                    "joint_2": [2, "ufactory-lite6"], # +/- 132 degrees
                    "joint_3": [3, "ufactory-lite6"], # -242 to 3.5 degrees
                    "joint_4": [4, "ufactory-lite6"], # +/- 360 degrees
                    "joint_5": [5, "ufactory-lite6"], # +/- 124 degrees
                    "joint_6": [6, "ufactory-lite6"], # +/- 360 degrees
                    "gripper": [7, "ufactory-lite6"],
                },
            ),
        },
        leader_arms={},
        cameras={},
    )

    robot = ManipulatorRobot(robot_config) # [cite: 2]
    robot.connect()
    robot.follower_arms['main'].api.set_mode(1)
    time.sleep(5)
    try:
        print("Moving to init pos")
        robot.follower_arms['main'].set_position(np.array([0, 0, 1, 0, 0, 0, 0, 0]))
        time.sleep(1)
        robot.follower_arms['main'].api.clean_error()
        robot.follower_arms['main'].api.clean_warn()
        
        print("Simple move")
        robot.follower_arms['main'].set_position(np.array([0, 0, 30, 0, 0, 0, 0, 0]))
        time.sleep(5)
        robot.follower_arms['main'].api.clean_error()
        robot.follower_arms['main'].api.clean_warn()
        
        print("Moving back to init pos")
        robot.follower_arms['main'].set_position(np.array([0, 0, 1, 0, 0, 0, 0, 0]))
        robot.follower_arms['main'].api.clean_error()
        robot.follower_arms['main'].api.clean_warn()
    except KeyboardInterrupt:
        print("Exiting")