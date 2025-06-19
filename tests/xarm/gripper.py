from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('192.168.1.224')
arm.connect()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
for _ in range(10):
    time.sleep(1)
    arm.open_lite6_gripper()
    print(arm.get_tgpio_digital())
    time.sleep(1)
    arm.close_lite6_gripper()
    print(arm.get_tgpio_digital())
arm.stop_lite6_gripper()