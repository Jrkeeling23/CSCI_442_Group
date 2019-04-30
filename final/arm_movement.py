import robot_control
import time
robot = robot_control.MoveRobot()
robot.close_hand()
time.sleep(1)
robot.open_hand()
time.sleep(1)

robot.rotate_hand_fun(7000)