# Alex Harry
# Justin Keeling
import threading
import time
import maestro

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3
# # for left arm
SHOULDER = 12
ARM_0 = 13
ELBOW = 14
ARM_2 = 15
ARM_3 = 16
HAND = 17
#right arm
# SHOULDER = 6
# ARM_0 = 7
# ELBOW = 8
# ARM_2 = 9
# ARM_3 = 10
# HAND = 11

class MoveRobot:

    def __init__(self):
        # Code help from source keyboardControl.py
        self.tango = maestro.Controller()
        # Default center values
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 1500
        self.motors = 6000
        self.turn = 6000
        self.shoulder = 6000
        self.arm_0 = 6000
        self.arm_2 = 6000
        self.arm_3 = 6000
        self.elbow = 6000
        self.hand = 6000
        self.center_robot()

    def stop(self):  # Stops the robot from moving or turning
        if self.motors > 6000:  # Slows the robot down slightly before completely stopping
            self.motors -= int((self.motors - 6000) / 2)
        elif self.motors < 6000:
            self.motors += int((6000 - self.motors) / 2)
        self.tango.setTarget(MOTORS, self.motors)
        if self.turn > 6000:
            self.turn -= int((self.turn - 6000) / 2)
        elif self.turn < 6000:
            self.turn += int((6000 - self.turn) / 2)
        self.tango.setTarget(TURN, self.turn)
        self.motors = 6000
        self.turn = 6000
        self.tango.setTarget(MOTORS, self.motors)  # Completely stops the robot
        self.tango.setTarget(TURN, self.turn)
        time.sleep(.5)

    def center_robot(self):  # Centers the robot and tilts the head down
        self.tango.setTarget(HEADTURN, self.headTurn)
        self.tango.setTarget(HEADTILT, self.headTilt)
        self.tango.setTarget(BODY, self.body)
        self.tango.setTarget(HAND, self.hand)
        self.tango.setTarget(ELBOW, self.elbow)
        self.tango.setTarget(SHOULDER, self.shoulder)
        self.tango.setTarget(ARM_0, self.arm_0)
        self.tango.setTarget(ARM_2, self.arm_2)
        self.tango.setTarget(ARM_3, self.arm_3)



    # Checks the limit for the wheels moving forward and backwards
    def forward_back_limit(self):
        if self.motors < 1510:
            self.motors = 1510
        elif self.motors > 7900:
            self.motors = 7900

    def wheels_forward(self):  # Moves the wheels forward
        self.motors -= 800
        self.forward_back_limit()
        self.tango.setTarget(MOTORS, self.motors)
        time.sleep(1)
        self.stop()

    def wheels_backward(self):  # Moves the wheels backwards
        self.motors += 800
        self.forward_back_limit()
        self.tango.setTarget(MOTORS, self.motors)
        time.sleep(1)
        self.stop()

    def turn_limit(self):  # Makes sure turn limits are in control
        if self.turn < 2110:
            self.turn = 2110
        elif self.turn > 7400:
            self.turn = 7400

    def turn_right(self):  # Turns robot right
        self.turn -= 1500
        self.turn_limit()
        self.tango.setTarget(TURN, self.turn)
        #    time.sleep(.2)
        time.sleep(.15)
        self.stop()
        print("turn right")

    def turn_left(self):  # Turns robot left
        self.turn += 1500
        self.turn_limit()
        self.tango.setTarget(TURN, self.turn)
        # time.sleep(.2)
        time.sleep(.15)
        self.stop()
        print("turn left")

    def move_head(self, h_val, v_val):  # method to move the robot head
        self.headTilt = self.check_motor_value(v_val, 1510, 7900)
        self.headTurn = self.check_motor_value(h_val, 1510, 7900)
        self.tango.setTarget(HEADTURN, self.headTurn)
        self.tango.setTarget(HEADTILT, self.headTilt)
        time.sleep(.1)

    def move_wheels(self, move, value):  # Method to move the wheels of the robot
        if move == "turn":
            self.turn = self.check_value_turn(value, 2110, 7400)
            self.tango.setTarget(TURN, self.turn)
        elif move == "move":
            self.motors = self.check_motor_value(value, 1510, 7900)
            self.tango.setTarget(MOTORS, self.motors)
        time.sleep(1)
        threading.Thread(target=self.stop())

    # # Methods to check value boundaries of servos
    # def check_value(self, val):
    #     if val > 7900:
    #         return 7900
    #     elif val < 1510:
    #         return 1510
    #     else:
    #         return val
    #
    # def check_value_turn(self, val):
    #     if val > 7400:
    #         return 7400
    #     elif val < 2110:
    #         return 2110
    #     else:
    #         return val

    def check_motor_value(self, val, min, max):
        if val > max:
            return max
        elif val < min:
            return min
        else:
            return val

    def get_inc(self, val, body_part):  # Gets increment value for moving part
        inc_value = 150
        if body_part < val:
            return inc_value
        else:
            return inc_value * -1

    def move_shoulder(self, val):  # Method to move shoulder
        # Lower value = higher shoulder 6000-4000
        self.shoulder= self.check_motor_value(val, 3900, 6100)
        self.tango.setTarget(SHOULDER, self.shoulder)
        # val = self.check_motor_value(val, 3900, 6100)
        # inc = self.get_inc(val, self.shoulder)
        # for i in range(self.shoulder, val, inc):
        #     self.shoulder = i
        #     self.tango.setTarget(SHOULDER, self.shoulder)
        #     time.sleep(.1)

    def move_arm_0(self, val):  # Method to move arm
        # Lower value = higher arm 6000-4000
        self.arm_0 = self.check_motor_value(val, 3900, 8100)
        self.tango.setTarget(ARM_0, self.arm_0)

        # inc = self.get_inc(val, self.arm_0)
        # for i in range(self.arm_0, val, inc):
        #     self.arm_0 = i
        #     self.tango.setTarget(ARM_0, self.arm_0)
        #     time.sleep(.1)

    def move_arm_2(self, val):  # Method to move arm
        # Lower value = higher arm 6000-4000
        self.arm_2 = self.check_motor_value(val, 3900, 6100)
        self.tango.setTarget(ARM_2, self.arm_2)
        # val = self.check_motor_value(val, 3900, 6100)
        # inc = self.get_inc(val, self.arm_2)
        # for i in range(self.arm_2, val, inc):
        #     self.arm_2 = i
        #     self.tango.setTarget(ARM_2, self.arm_2)
        #     time.sleep(.1)

    def move_arm_3(self, val):  # Method to move arm
        # Lower value = higher arm 6000-4000
        # val = self.check_motor_value(val, 3900, 6100)
        self.arm_3 = self.check_motor_value(val, 3900, 6100)
        self.tango.setTarget(ARM_3, self.arm_3)
        # inc = self.get_inc(val, self.arm_3)
        # for i in range(self.arm_3, val, inc):
        #     self.arm_3 = i
        #     self.tango.setTarget(ARM_3, self.arm_3)
        #     time.sleep(.1)


    def move_elbow(self, val):  # Method to move elbow
        # Lower value = higher shoulder 7000-4000
        self.elbow = self.check_motor_value(val, 3900, 8100)
        self.tango.setTarget(ELBOW, self.elbow)
        # val = self.check_motor_value(val, 3900, 8100)
        # inc = self.get_inc(val, self.elbow)
        # for i in range(self.elbow, val, inc):
        #     self.elbow = i
        #     self.tango.setTarget(ELBOW, self.elbow)
        #     time.sleep(.1)

    def move_hand(self, val):  # Method to move hand
        # Lower value = higher 8000-4000
        self.hand = self.check_motor_value(val, 3900, 8100)
        self.tango.setTarget(HAND, self.hand)
        # val = self.check_motor_value(val, 3900, 8100)
        # inc = self.get_inc(val, self.hand)
        # for i in range(self.hand, val, inc):
        #     self.hand = i
        #     self.tango.setTarget(HAND, self.hand)
        #     time.sleep(.1)


    def close_hand(self):
        self.move_hand(8300)

    def open_hand(self):
        self.move_hand(4000)


    def arm_in_cam_view(self):
        threading.Thread(target=self.move_shoulder(3900)).start()
        threading.Thread(target=self.move_arm_0(7900)).start()
        threading.Thread(target=self.move_elbow(8200)).start()
        threading.Thread(target=self.move_arm_2(6200)).start()
        threading.Thread(target=self.move_arm_3(2000)).start()

    def lower_arm(self):
        threading.Thread(target=self.move_shoulder(6000)).start()
        threading.Thread(target=self.move_arm_0(6000)).start()
        threading.Thread(target=self.move_elbow(6000)).start()
        threading.Thread(target=self.move_arm_2(6000)).start()
        threading.Thread(target=self.move_arm_3(6000)).start()