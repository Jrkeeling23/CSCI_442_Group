import robot_control
import time
robot = robot_control.MoveRobot()

robot.arm_in_cam_view()
time.sleep(3)
robot.close_hand()
time.sleep(3)
robot.drop()