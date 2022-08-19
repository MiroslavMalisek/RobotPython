from pyrobot.brain import Brain

class MyAvoid(Brain):

    def setup(self):
        print("Empezamos")

    def estado_def(self, front, left, right):
        if (left > 0.4) and (left < 0.6):
            return "left"
        elif (right > 0.4) and (left < 0.6):
            return "right"
        elif (front > 0.4) and (front < 0.6):
            return "front"
        else:
            return "no_estado"

    def determineMove(self, estado, front, left, right):
        if estado == 0:
            if front < 0.5:
                # print("obstacle ahead, hard turn")
                return (0, -0.3)
            elif left < 0.5:
                # print("object detected on left, slow turn")
                return (0.0, -0.1)
            elif right < 0.5:
                # print("object detected on right, slow turn")
                return (0.0, 0.3)
            else:
                # print("clear")
                return (0.5, 0.0)
        if estado == 1:
            if front < 0.5:
                # print("obstacle ahead, hard turn")
                return (0, 0.3)
            elif left < 0.5:
                # print("object detected on left, slow turn")
                return (0.0, -0.3)
            elif right < 0.5:
                # print("object detected on right, slow turn")
                return (0.0, 0.1)
            else:
                # print("clear")
                return (0.5, 0.0)
        if estado == 2:
            if left < 0.5:
                # print("object detected on left, slow turn")
                return (0.0, -0.3)
            elif right < 0.5:
                # print("object detected on right, slow turn")
                return (0.0, 0.3)
        if estado == 3:
            if (left < 0.5) or (right < 0.5) or (front < 0.5):
                # print("object detected on left, slow turn")
                return (0.0, 0.0)
            elif right < 0.5:
                # print("object detected on right, slow turn")
                return (0.5, 0.0)


    def step(self):
        front = min([s.distance() for s in self.robot.range['front']])
        left = min(self.robot.range[1].distance(), self.robot.range[2].distance(), self.robot.range[3].distance())
        right = min(self.robot.range[4].distance(), self.robot.range[5].distance(), self.robot.range[6].distance())

        if (left > 0.4) and (left < 0.6):
            state = 0
        elif (right > 0.4) and (left < 0.6):
            state = 1
        elif (front > 0.4) and (front < 0.6):
            state = 2
        else:
            state = 3
        #state = estado_def(front, left, right)
        translation, rotate = self.determineMove(state, front, left, right)
        self.robot.move(translation, rotate)



def INIT(engine):
    assert engine.robot.requires("range-sensor") and engine.robot.requires(
        "continuous-movement"
    )
    return MyAvoid("MyAvoid", engine)
