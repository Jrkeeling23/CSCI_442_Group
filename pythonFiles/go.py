import robot_control as control

move = control.MoveRobot()
move.center_robot()
move.wheels_forward()
move.wheels_backward()
move.turn_right()
move.turn_left()
