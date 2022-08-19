from pyrobot.brain import *
from pyrobot.map.lps import LPS
from pyrobot.map.gps import GPS
from pyrobot.map import Map
import _thread


class Mapping(Brain):

    TOLERANCE = 1.0

    def setup(self):
        # We want our map to measure in MM, so we first store
        # our current unit of measure
        units = self.robot.range.units
        # We then reset our measurements to MMs
        self.robot.range.units = "MM"
        # Calculate the maximum range of our sensors
        rangeMaxMM = self.robot.range.getMaxvalue()
        sizeMM = rangeMaxMM * 2 + (self.robot.radius / 1000.0)  # in MMs
        # Reset our unit of measure
        self.robot.range.units = units
        # Now, we create our Local Perceptual Space window -
        # this will hold our local map Map will be 20px by 20px
        # and will represent a height and width of sizeMM (total
        # sensor range)
        self.lps = LPS(80, 80, widthMM=sizeMM, heightMM=sizeMM)
        # Then create our Global Perceptual Space window - this
        # will hold our global map This map will be 100px by
        # 100px and will represent an area 10 times the size of
        # our maximum range
        self.gps = GPS(
            cols=100,
            rows=100,
            gridUpdate="MatthiesElfes",
            #                                gridUpdate="Thrun93",
            #                                 gridUpdate="Konolige",
            heightMM=float(sizeMM * 1),
            widthMM=float(sizeMM * 1),
        )

        self.needCompleteRedraw = True
        self.needRedraw = False
        self.lock = _thread.allocate_lock()

    def step(self):
        if not self.lock.acquire(False):
            return

        # First we clear out all our old LPS data
        self.lps.reset()
        # Next we update our LPS with current 'range' sensor readings
        self.lps.sensorHits(self.robot, "range")

        # Then update our GPS with the new information in the LPS
        self.gps.updateFromLPS(self.lps, self.robot)

        # Finally, we redraw
        self.needRedraw = True
        self.lock.release()

        left = min([s.distance() for s in self.robot.range["left"]])
        right = min([s.distance() for s in self.robot.range["right"]])
        front = min([s.distance() for s in self.robot.range["front"]])

        # print("left", left, "front", front, "right", right)

        if left < self.TOLERANCE and right < self.TOLERANCE:
            self.robot.move(0, 0.2)
        elif right < self.TOLERANCE:
            self.robot.move(0, 0.2)
        elif left < self.TOLERANCE:
            self.robot.move(0, -0.2)
        elif front < self.TOLERANCE:
            self.robot.move(0, 0.2)  # arbitrarily turn one way
        else:
            self.robot.move(0.2, 0)

    def redraw(self):
        if not self.lock.acquire(False):
            return
        if self.needRedraw:
            self.lps.redraw(drawLabels=False)
            self.gps.update()
            if self.needCompleteRedraw:
                self.gps.redraw()
                self.needCompleteRedraw = False
            else:
                self.gps.redraw(onlyDirty=1)
        self.needRedraw = False
        self.lock.release()

    def destroy(self):
        # Make sure we close down cleanly
        self.lps.destroy()
        self.gps.destroy()


def INIT(engine):
    assert engine.robot.requires("range-sensor") and engine.robot.requires(
        "continuous-movement"
    )

    # If we are allowed (for example you can't in a simulation), enable
    # the motors.
    try:
        engine.robot.position[0]._dev.enable(1)
    except AttributeError:
        pass

    return Mapping("Mapping Brain", engine)
