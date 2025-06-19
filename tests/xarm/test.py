from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('192.168.1.224', baud_checkset=False) # Replace with your robot's IP address
arm.connect()
arm.motion_enable(enable=True)
arm.set_state(0) # Set to ready state
arm.set_mode(2) # Set to manual mode
arm.set_tgpio_digital(ionum=2, value=1) # Required to set to manual mode
try:
    while True:
        time.sleep(1)
        print(arm.get_servo_angle())
except KeyboardInterrupt:
    arm.disconnect()