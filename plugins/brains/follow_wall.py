# robot goes forward and then slows to a stop when it detects something

from pyrobot.brain import Brain


class Avoid(Brain):

    def setup(self):
        self.mode = False  # True - following, False - picking behavior
        self.wall = -1  # -1 - no wall, 1 - left, 2 - right
        #self.get_angle_before()
        #self.get_angle_after()

    def too_close(self, distance):
        if distance < 0.45:
            return True
        else:
            return False

    def too_far(self, distance):
        if distance >= 0.55:
            return True
        else:
            return False
    """    
    def get_angle_before(self):
        self.angle_before = int(self.robot.th)
        if self.angle_before < 0:
            self.angle_before = 360 + self.angle_before

    def get_angle_after(self):
        self.angle_after = int(self.robot.th)
        if self.angle_after < 0:
            self.angle_after = 360 + self.angle_after
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
            #print2(" === Picking === ")
            translation, rotate = self.pick_wall(front, left_front, right_front)
        #translation, rotate = self.determineMove(front, left, right)
        self.robot.move(translation, rotate)
        #self.get_angle_after()
        """
        print(self.angle_before)
        print(self.angle_after)
        if self.angle_after == self.angle_before:
            self.mode = False
            self.wall = -1
        """

    def follow_wall(self, front, left_front, right_front):
        # check if there is no obstacle
        if (front < 0.7):
            print("Obstacle, stop following")
            self.mode = False
            self.wall = -1
            #self.get_angle_before()
            return(0, 0)
        if self.wall == 1:
            if self.too_far(left_front):
               return (0.1, 0.3)
            elif self.too_close(left_front):
                return (0.1, -0.3)
            else:
                return (0.4, 0.0)
        elif self.wall == 2:
            if self.too_far(right_front):
               return (0.1, -0.3)
            elif self.too_close(right_front):
                return (0.1, 0.3)
            else:
                return (0.4, 0.0)



    def pick_wall(self, front, left_front, right_front):

        if (front < 0.7) and (left_front < 0.5) and (right_front > 0.5):
            print("Turning right")
            return (0.0, -0.3)
        elif (front > 0.7) and (left_front > 0.5) and (right_front > 0.5):
            print("Going forward")
            #print(self.robot.th)
            return (0.5, 0.0)
        elif (front < 0.7) and (left_front > 0.5) and (right_front < 0.5):
            print("Turning left")
            return (0.0, 0.3)
        elif (front < 0.7) and (left_front > 0.5) and (right_front > 0.5):
            print("Obstacle, turning right")
            return (0.0, -0.3)
        elif (left_front < 0.5):
            self.wall = 1
            self.mode = True
            return (0, 0)
        elif (right_front < 0.5):
            self.wall = 2
            self.mode = True
            return (0, 0)



def INIT(engine):
    assert engine.robot.requires("range-sensor") and engine.robot.requires(
        "continuous-movement"
    )
    return Avoid("follow_wall", engine)
