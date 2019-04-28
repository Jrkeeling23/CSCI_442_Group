import robot_control

robot = robot_control.MoveRobot()
# robot.arm_in_cam_view()


for i in range(0, 12000, 150):
    robot.move_elbow(i)
    print(i)