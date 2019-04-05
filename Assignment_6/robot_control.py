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


class MoveRobot:

    def __init__(self):
        # Code help from source keyboardControl.py
        self.tango = maestro.Controller()
        # Default center values
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.center_robot()

    def stop(self):  # Stops the robot from moving or turning
        if self.motors > 6000:  # Slows the robot down slightly before completely stopping
            self.motors -= int((self.motors - 6000) / 2)
        elif self.motors < 6000:
            self.motors += int((6000 - self.motors) / 2)
        self.tango.setTarget(MOTORS, self.motors)
        if self.turn > 6000:
            self.turn -= int((self.turn - 6000))
        elif self.turn < 6000:
            self.turn += int((6000 - self.turn))
        self.tango.setTarget(TURN, self.turn)
        self.motors = 6000
        self.turn = 6000
        self.tango.setTarget(MOTORS, self.motors)  # Completely stops the robot
        self.tango.setTarget(TURN, self.turn)
        time.sleep(.1)

    def center_robot(self):  # Ctenters the robot and tilts the head down
        self.tango.setTarget(HEADTURN, self.headTurn)
        self.tango.setTarget(HEADTILT, 1510)
        self.tango.setTarget(BODY, self.body)

    def forward_back_limit(self):  # Checks the limit for the wheels moving forward and backwards
        if self.motors < 1510:
            self.motors = 1510
        elif self.motors > 7900:
            self.motors = 7900

    def wheels_forward(self):  # Moves the wheels forward
        self.motors -= 800
        self.forward_back_limit()
        self.tango.setTarget(MOTORS, self.motors)
        time.sleep(1.4)
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
        self.turn -=1500
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
        #time.sleep(.2)
        time.sleep(.15)
        self.stop()
        print("turn left")

    def move_head(self, h_val, v_val):  # method to move the robot head
        self.headTilt = self.check_value(v_val)
        self.headTurn = self.check_value(h_val)
        self.tango.setTarget(HEADTURN, self.headTurn)
        self.tango.setTarget(HEADTILT, self.headTilt)
        #time.sleep(.5)

    def move_wheels(self, move, value):  # Method to move the wheels of the robot
        if move == "turn":
            self.turn = self.check_value_turn(value)
            self.tango.setTarget(TURN, self.turn)
        elif move == "move":
            self.motors = self.check_value(value)
            self.tango.setTarget(MOTORS, self.motors)
        time.sleep(1)
        threading.Thread(target=self.stop())

    # Methods to check value boundaries of servos
    def check_value(self, val):
        if val > 7900:
            return 7900
        elif val < 1510:
            return 1510
        else:
            return val

    def check_value_turn(self, val):
        if val > 7400:
            return 7400
        elif val < 2110:
            return 2110
        else:
            return val
