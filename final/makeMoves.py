import robot_control
class Move:

    def __init__(self, width, height):
        self.forward_val = int(height * .8)
        self.turn_left_val = int(width * 0.55)
        self.turn_right_val = int(width * 0.45)
   #     self.robot = robot_control.MoveRobot()

    def decide_move(self,x_val, y_val):
        print(self.forward_val)
        print(y_val)
        if x_val is not None:
            if x_val < self.turn_right_val:
                print("turn right")
                return
            elif x_val > self.turn_left_val:
                print("turn left")
                return
        if y_val is not None and y_val < self.forward_val:
            print("move Forward")
