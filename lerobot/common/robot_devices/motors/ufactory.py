"""
Taken from: https://github.com/vmayoral/lerobot.git
"""

import enum
import threading
import time
import math
import numpy as np
from xarm.wrapper import XArmAPI

from lerobot.common.robot_devices.motors.configs import XarmMotorsBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError


class TorqueMode(enum.Enum):
    ENABLED = 1
    DISABLED = 0


class XArmWrapper:
    """Wrapper for the xArm Python SDK"""

    def __init__(
        self,
        config: XarmMotorsBusConfig,
    ):
        print("Initializing XArmWrapper")  # Debug print
        self.port = config.port
        self.motors = config.motors
        self.mock = config.mock

        self.calibration = None
        self.is_connected = False
        self.logs = {}

        self.api = None

        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500

        self.gripper_state = 0 # 0: Closed, 360: Opened

        # Stop event
        self.stop_event = threading.Event()

        # Create and start the digital input monitoring thread
        print("Creating monitor thread")  # Debug print
        self.monitor_input_thread = threading.Thread(
            target=self.monitor_digital_input, args=(self.stop_event,)
        )

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    def connect(self):
        print("Connecting to xArm")  # Debug print
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                f"DynamixelMotorsBus({self.port}) is already connected. Do not call `motors_bus.connect()` twice."
            )

        if self.mock:
            print("Mock mode, not connecting to real device")  # Debug print
            return
        else:
            self.api = XArmAPI(self.port)

        try:
            if not self.api.connected:
                raise OSError(f"Failed to connect to xArm API @ '{self.port}'.")
            print("Successfully connected to xArm")  # Debug print
        except Exception as e:
            print(f"Exception while connecting in XArmWrapper: {e}")
            raise

        # Allow to read and write
        self.is_connected = True

        # Start the monitoring thread after successful connection
        self.monitor_input_thread.start()
        print("Monitor thread started")  # Debug print

    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        pass  # TODO (@vmayoral): implement if of interest

    def read(self, data_name, motor_names: str | list[str] | None = None):
        pass  # TODO (@vmayoral): implement if of interest

    def enable(self, follower: bool = False):
        #NOTE: We do not enable gripper here because that API is not working for Lite6
        self.api.motion_enable(enable=True)
        self.api.clean_error()
        self.api.clean_warn()
        if follower:
            self.api.set_mode(1)
        else:
            self.api.set_mode(0)
            # Light up the digital output 2 (button), to signal manual mode
            self.api.set_tgpio_digital(ionum=2, value=1)
        self.api.set_state(0)
        self.api.close_lite6_gripper()
        self.gripper_state = 0

    def disconnect(self):
        print("Disconnecting from xArm")  # Debug print
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"FeetechMotorsBus({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )

        # Turn off manual mode after recording
        self.api.set_mode(0)
        self.api.set_state(0)
        # Light down the digital output 2 (button), to signal manual mode
        # self.api.set_tgpio_digital(ionum=2, value=0)
        # Disconnect both arms
        self.api.disconnect()

        # Stop events and threads
        self.stop_event.set()
        print("Waiting for monitor thread to join")  # Debug print
        self.monitor_input_thread.join()
        print("Monitor thread joined")  # Debug print

        # Signal as disconnected
        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()

    def get_position(self):
        code, angles = self.api.get_servo_angle()
        angles = angles[:6] # For XArm Lite 6, there's only 6 DoF. The API returns 7 values, with the 7th being always zero
        pos = angles + [self.gripper_state]
        return pos

    def set_position(self, position: np.ndarray):
        angles = position[:-1].tolist() + [0] # Add back the 7th (zero) value so that the API does not throw error
        gripper_state = int(position[-1])

        # joints
        # print(angles)
        # self.api.set_servo_angle(angles=angles, is_radian=False, wait=False, speed=self._tcp_speed, mvacc=self._tcp_acc)
        self.api.set_servo_angle_j(angles=angles, is_radian=False, wait=False, speed=self._tcp_speed, mvacc=self._tcp_acc)

        # gripper 
        if np.abs(360 - gripper_state) < np.abs(0 - gripper_state): # Check gripper_state closer to which number, 0 or 360. If closer to 360, open, else close
            self.api.open_lite6_gripper()
        else:
            self.api.close_lite6_gripper()

    def monitor_digital_input(self, stop_event):
        """
        Short click once to open gripper, and click again to close gripper.
        Long press once to start manual mode, and long press again to stop manual mode
        """
        print("Starting monitor_digital_input")  # Debug print
        long_click_time = 1.0

        last_press_time = 0
        long_click_detected = False
        # click_count = 0
        long_click_state = False  

        while not stop_event.is_set():
            try:
                if self.api is not None and self.is_connected:
                    code, value = self.api.get_tgpio_digital(ionum=2)
                    # print(f"Digital input read: code={code}, value={value}")  # Debug print
                    if code == 0:
                        current_time = time.time()

                        if value == 1:  # Button pressed
                            if last_press_time == 0:
                                last_press_time = current_time
                            elif (not long_click_detected and current_time - last_press_time >= long_click_time):
                                long_click_detected = True
                                long_click_state = not long_click_state
                                if long_click_state:
                                    print("Long click detected -> Start manual mode")
                                    self.api.set_tgpio_digital(ionum=2, value=1)
                                    # manual mode
                                    self.api.clean_error()
                                    self.api.set_mode(2)
                                    self.api.set_state(0)
                                else:
                                    print("Long click detected -> Stop manual mode")
                                    self.api.set_tgpio_digital(ionum=2, value=0)
                                    # disable manual mode
                                    self.api.clean_error()
                                    self.api.set_mode(0)
                                    self.api.set_state(0)
                        else:  # Button released
                            if last_press_time != 0:
                                if not long_click_detected:
                                    if self.gripper_state: # Gripper is opened, let's close it 
                                        print("Short click detected -> Close gripper")
                                        self.api.close_lite6_gripper()
                                        self.gripper_state = 0
                                    else: # Gripper is closed, let's open it
                                        print("Short click detected -> Open gripper")
                                        self.api.open_lite6_gripper()
                                        self.gripper_state = 360

                                last_press_time = 0
                                long_click_detected = False


                else:
                    print("API not connected, waiting...")
                    time.sleep(1)  # Wait a bit longer before checking again
            except Exception as e:
                print(f"Error in monitor_digital_input: {e}")  # Debug print
            time.sleep(0.01)  # Check every 10ms for more precise detection
        print("Exiting monitor_digital_input")  # Debug print

    def robot_reset(self):
        """Reset the robot to a safe state"""
        self.api.set_gripper_position(pos=600, wait=True)  # Open gripper
