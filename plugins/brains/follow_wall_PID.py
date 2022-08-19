# robot goes forward and then slows to a stop when it detects something

from pyrobot.brain import Brain


class Avoid(Brain):

    def setup(self):
        self.mode = False  # True - following, False - picking wall
        self.wall = -1  # -1 - no wall, 1 - left, 2 - right
        self.outer_corner = 0   #0 - not in outer corner, 1 - in outer corner
        self.setpoint = 0.5     #default distance from the wall

        self.Kp = 1.4
        self.Ki = 0.01
        self.Kd = 0.0
        self.integralArea = 0.0
        self.previousError = 0.0

        """
        1.5
        0.01
        0.7
        
        self.Kp = 1.5
        self.Ki = 0.02
        self.Kd = 2.0
        """

    def step(self):
        front = min([s.distance() for s in self.robot.range["front"]])
        left_front = min([s.distance() for s in self.robot.range["left-front"]])
        right_front = min([s.distance() for s in self.robot.range["right-front"]])
        if self.mode:
            # Following wall
            translation, rotate = self.follow_wall(front, left_front, right_front)
        else:
            # Find wall
            translation, rotate = self.pick_wall(front, left_front, right_front)
        self.robot.move(translation, rotate)



    def follow_wall(self, front, left_front, right_front):
        # check if there is no obstacle
        if (front < 0.7):
            print("Obstacle, stop following")
            self.mode = False   #change mode to picking wall
            self.wall = -1      #change to no wall
            self.integralArea = 0       #reset the sum of integral
            self.previousError = 0
            return(0, 0)
        if self.wall == 1:          #check if the robot follows left hand side wall
            print("Following left...")
            if self.robot.range[1].distance() > 2.0:    #check if robot came to outer corner
                print("In the corner...")
                self.integralArea = 0       #reset
                self.previousError = 0      #reset
                self.Kd = 1.5               #change value of Kd to higher value
                self.Kp = 6.0               #same for Kp - we want the robot to take the corner sharply, the closest as possilbe
                self.outer_corner = 1       #change value to "in the corner"
            if (self.robot.range[2].distance() < 1.0) and (self.outer_corner == 1):     #check if robot is leaving the corner
                print("Out of corner...")
                self.integralArea = 0       #reset
                self.previousError = 0      #reset
                self.Kp = 1.2               #restore the default value
                self.Ki = 0.0              #restore the default value
                self.Kd = 0.4               #restore the default value
                self.outer_corner = 0       #change value to "out of the corner"
            #print(left_front)
            error = self.setpoint - left_front      #calculate the actual error between the desired distance and actual distnce
            self.integralArea += error              #integral part of PID control
            correction = (self.Kp * error + self.Ki * self.integralArea + self.Kd * (self.previousError - error))   #calculation of PID contol
            self.previousError = error      #save the actual value of error in order to use it later
            if (correction < 0.1 and correction > -0.1):    #if the result of PID is very small, we use 0 instead
                correction = 0
            turn = min(max(-1.0, 0.0 - correction), 1.0)        #calculate turn ratio (value can be between -1, 1)
            speed = max(min(1.0, 1.0 - abs(correction)), 0.0)   #calculate speed ratio (value can be between -1, 1)
            return(speed, turn)
        elif self.wall == 2:        #check if the robot follows right hand side wall
            print("Following right...")
            if self.robot.range[6].distance() > 2.0:  # check if robot came to outer corner
                print("In the corner...")
                self.integralArea = 0  # reset
                self.previousError = 0  # reset
                self.Kd = 1.5  # change value of Kd to higher value
                self.Kp = 6.0  # same for Kp - we want the robot to take the corner sharply, the closest as possilbe
                self.outer_corner = 1  # change value to "in the corner"
            if (self.robot.range[5].distance() < 1.0) and (self.outer_corner == 1):  # check if robot is leaving the corner
                print("Out of corner...")
                self.integralArea = 0  # reset
                self.previousError = 0  # reset
                self.Kp = 2.0  # restore the default value
                self.Ki = 0.0  # restore the default value
                self.Kd = 0.3  # restore the default value
                self.outer_corner = 0  # change value to "out of the corner"
            # print(left_front)
            error = self.setpoint - right_front  # calculate the actual error between the desired distance and actual distnce
            self.integralArea += error  # integral part of PID control
            correction = (self.Kp * error + self.Ki * self.integralArea + self.Kd * (self.previousError - error))  # calculation of PID contol
            self.previousError = error  # save the actual value of error in order to use it later
            if (correction < 0.1 and correction > -0.1):  # if the result of PID is very small, we use 0 instead
                correction = 0
            turn = min(max(-1.0, 0.0 + correction), 1.0)  # calculate turn ratio (value can be between -1, 1)
            speed = max(min(1.0, 1.0 - abs(correction)), 0.0)  # calculate speed ratio (value can be between -1, 1)
            return (speed, turn)



    def pick_wall(self, front, left_front, right_front):
        if (front < 0.7) and (left_front < 0.6) and (right_front > 0.6):
            print("Turning right...")
            while min([s.distance() for s in self.robot.range["front"]]) < 2.0:
                self.robot.move(0.0, -0.5)
            return (0.0, 0.0)
        elif (front > 0.7) and (left_front > 0.6) and (right_front > 0.6):
            print("Going forward...")
            return (1.0, 0.0)
        elif (front < 0.7) and (left_front > 0.6) and (right_front < 0.6):
            print("Turning left...")
            while min([s.distance() for s in self.robot.range["front"]]) < 2.0:
                self.robot.move(0.0, 0.5)
            return (0.0, 0.0)
        elif (front < 0.7) and (left_front > 0.6) and (right_front > 0.6):
            print("Obstacle, turning left...")
            return (0.0, 0.5)
        elif (left_front <= 0.6):
            self.wall = 1
            self.mode = True
            return (0, 0)
        elif (right_front <= 0.6):
            self.wall = 2
            self.mode = True
            return (0, 0)


def INIT(engine):
    assert engine.robot.requires("range-sensor") and engine.robot.requires(
        "continuous-movement"
    )
    return Avoid("follow_wall_PID", engine)
