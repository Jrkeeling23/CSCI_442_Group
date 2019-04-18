import robot_control

robot = robot_control.MoveRobot()
robot.move_shoulder_vertically(4000)


# for i in range(4000, 8100, 150):
#     robot.move_hand(i)
#     print(i)