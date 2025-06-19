"""
Date: 13/06/2025
Desc: To test connection to UFactory Lite 6 and monitor leader arm manual movement
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
        leader_arms={
            "main": XarmMotorsBusConfig(
                port="192.168.1.153",  # <<-- UPDATE THIS PORT
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
        follower_arms={},
        cameras={},
    )

    robot = ManipulatorRobot(robot_config) # [cite: 2]
    robot.connect()
    # robot.leader_arms['main'].api.set_mode(6)
    try:
        while True:
            time.sleep(1)
            print(f"Position: {np.array(robot.leader_arms['main'].get_position())}")
    except KeyboardInterrupt:
        print("Operation interrupted by user.")

    robot.disconnect()
