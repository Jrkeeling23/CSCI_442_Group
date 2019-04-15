import robot_control
class Move:

    def __init__(self, width, height):
        self.forward_val = int(height * .8)
        self.turn_left_val = int(width * 0.55)
        self.turn_right_val = int(width * 0.45)
        self.robot = robot_control.MoveRobot()

    def decide_move(self,x_val, y_val): # Method decides where the robot will move
        if x_val is not None: # Makes sure x is a value
            if x_val < self.turn_right_val:
                #print("turn right")
 #               self.robot.turn_right()
                return
            elif x_val > self.turn_left_val:
                #print("turn left")
  #              self.robot.turn_left()
                return
            elif self.robot.turn != 6000:
   #             self.robot.stop()
                pass
        if y_val is not None: # makes sure y is value
            #print("move Forward")
            if y_val < self.forward_val:
               pass
               # self.robot.wheels_forward()
            elif self.robot.motors != 6000:
                #self.robot.stop()
                pass