from pyrobot.map.tkmap import TkMap
from pyrobot.map.occupancyGrid import OccupancyGrid
import math

try:
    import pyrobot.system.share as share
except:

    class share:
        debug = 0

class GPS(OccupancyGrid):
    """
   GUI for visualizing the global perceptual space of a robot.
   """

    """
   ---------------------------------
   __init__: initialize a GPS window
   ----------------------------------
     cols: number of columns in map
     rows: number of rows in map
     value: default value for the cells
     widthMM: total width which we want to represent in the map (in MMs)
     heightMM: total height which we want to represent in the map (in MMs)
   """

    def __init__(
        self, cols=400, rows=400, widthMM=10000, heightMM=10000, gridUpdate=""
    ):

        # max probability of occupancy
        self.maxOccupied = 0.98
        self.minOccupied = 0.001

        # Thresholds of certainty
        self.occThreshold = 0.8
        self.emptyThreshold = 0.2

        # create the occupancy grid
        OccupancyGrid.__init__(
            self,
            cols=cols,
            rows=rows,
            widthMM=widthMM,
            heightMM=heightMM,
            gridResize=4,
            gridUpdate=gridUpdate,
        )

    """
   -------------------------------------------------------------------------
   updateFromLPS: update values based on LPS
   -------------------------------------------------------------------------
     lps: class containing sensor data
     robot: contains information about current robot
   """

    def updateFromLPS(self, lps, robot):

        # First we get the robot's current location in MM
        robotXmm = robot.x * 1000
        robotYmm = robot.y * 1000
        robotThr = robot.thr
        #      print "Starting updateFromLPS, current pose: %f, %f, %f " %
        #            (robot.x,robot.y,robot.thr)

        # We need to remember that the LPS has a different scale, so
        # depending on the relationship between the two scales it is
        # possible that there could be a cell with a hit and another
        # adjacent cell empty which both map to the same cell in the GPS
        # In this case we want to give priority to the hit.  So we collect
        # all the updates and apply the empty changes first and then the
        # hit changes.

        emptyCells = []
        hitCells = []

        # cycle through the grid of sonar readings in the LPS
        for currentRow in range(lps.rows):
            for currentCol in range(lps.cols):

                # if the lps doesn't have any info on this cell, skip
                # directly to the next cell
                if (
                    lps.getGridLocation(row=currentRow, col=currentCol, absolute=1)
                    == 0.5
                ):
                    continue

                # shouldn't it be y which is negated??
                sensorXmm = -1 * (currentRow - (lps.rows / 2)) * lps.rowScaleMM
                sensorYmm = (currentCol - (lps.cols / 2)) * lps.colScaleMM

                # compensate for the robot's rotation
                # remember cos(0) = 1, sin(0) = 0
                colOffMM = sensorXmm * math.cos(robotThr) - sensorYmm * math.sin(
                    robotThr
                )
                rowOffMM = sensorXmm * math.sin(robotThr) + sensorYmm * math.cos(
                    robotThr
                )

                # current location in question
                colHitMM = robotXmm + colOffMM
                rowHitMM = robotYmm + rowOffMM

                # current cell in question
                colCell = int(colHitMM / self.colScaleMM)
                rowCell = self.cols - int(rowHitMM / self.rowScaleMM) - 1

                # robot's position, assume it started in the center of the
                # grid
                colCentered = int(colCell + self.cols / 2)
                rowCentered = int(rowCell - self.rows / 2)

                # current location value
                oldVal = self.getGridLocation(
                    row=rowCentered, col=colCentered, absolute=1
                )

                # the value in the lps
                newInfo = lps.getGridLocation(
                    row=currentRow, col=currentCol, absolute=1
                )
                # we combine the two to calculate the new probability
                occ = self.getProb(oldVal, newInfo)

                #            print "x,y hit (%f,%f,%f,%f) in cell (%d,%d) now %f" % \
                #                  (colOffMM,rowOffMM,colHitMM,rowHitMM,colCentered,
                #                   rowCentered, occ)

                if occ > self.occThreshold:
                    hitCells.append((rowCentered, colCentered, occ))
                else:
                    emptyCells.append((rowCentered, colCentered, occ))

        # apply the changes to the grid while giving preference to
        # hit cells
        for (rowCentered, colCentered, newVal) in emptyCells:
            self.updateCell(rowCentered, colCentered, newVal)
        for (rowCentered, colCentered, newVal) in hitCells:
            self.updateCell(rowCentered, colCentered, newVal)

    #      self.display()
    #      print "--- One updateFromLPS done ---"

    """
   -------------------------------------------------------------------------
   updateCell: update the grid location with the new value and mark that
               location for redraw in the next redraw cycle
   -------------------------------------------------------------------------
   """

    def updateCell(self, row, col, value):
        # get the current value
        oldVal = self.getGridLocation(row=row, col=col, absolute=1)

        # no need to really update if value isn't changing
        if value != oldVal:
            # we want to keep in the range 0 - 1 but NOT too small or big
            # in case things change
            if value < self.minOccupied:
                value = self.minOccupied
            elif value > self.maxOccupied:
                value = self.maxOccupied

            # give the cell its new value
            self.setGridLocation(row=row, col=col, value=value, absolute=1)
            self.addDirtyCells(row=row, col=col)


if __name__ == "__main__":
    gps = GPS(50, 50)
    gps.application = 1
    gps.mainloop()
