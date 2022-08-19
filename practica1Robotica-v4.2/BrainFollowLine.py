from pyrobot.brain import Brain

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyrobot.tools.followLineTools import findLineDeviation

class BrainFollowLine(Brain):

  #parameter to work with while following the line
  NO_ERROR = 0
  integralArea_line = 0.0
  previousError_line = 0.0
  Kp_line = 1.6
  Ki_line = 0.03
  Kd_line = 0.1
  foundLine = False
  errorLine = 0.0

  #parameters to work with when there is a obstacle
  modeWall = False  # True - following, False - no wall
  wall = -1  # -1 - no wall, 1 - left, 2 - right
  outer_corner = 0  # 0 - not in outer corner, 1 - in outer corner
  setpoint = 0.5  # default distance from the wall
  Kp_obstacle = 1.4
  Ki_obstacle = 0.0
  Kd_obstacle = 0.2
  integralArea_obstacle = 0.0
  previousError_obstacle = 0.0

  #sonares
  front = 0.0
  left_front = 0.0
  right_front = 0.0

  def setup(self):
    self.image_sub = rospy.Subscriber("/image",Image,self.callback)
    self.bridge = CvBridge()

  def callback(self,data):
    self.rosImage = data

  def destroy(self):
    cv2.destroyAllWindows()

  def get_image(self):
    # take the last image received from the camera and convert it into
    # opencv format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(self.rosImage, "bgr8")
    except CvBridgeError as e:
      print(e)
    # display the robot's camera's image using opencv
    cv2.imshow("Stage Camera Image", cv_image)
    cv2.waitKey(1)
    # convert the image into grayscale
    imageGray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # determine the robot's deviation from the line.
    self.foundLine, self.errorLine = findLineDeviation(imageGray)


  def check_wall(self):
    if (min([s.distance() for s in self.robot.range["front"]]) <= 0.3):   #check for obstacle in front of the robot
      self.modeWall = True
      print("Obstacle, turning right...")
      while min([s.distance() for s in self.robot.range["front"]]) < 3.0:   #turn the robot ther is no obstacle in front of him
        self.robot.move(0.0, -0.5)
    else:
      self.modeWall = False


  def avoid_obstacle(self):
    # check if there is no line, if yes, stop avoiding
    self.get_image()
    if self.foundLine:
      print("Line found...")
      self.modeWall = False  # change mode to not following wall
      #self.wall = -1  # change to no wall
      self.integralArea_obstacle = 0  # reset the sum of integral
      self.previousError_obstacle = 0
      return
    print("Following left...")
    if (self.robot.range[0].distance() > 3.0) and (self.outer_corner == 0):  # check if robot came to outer corner
      print("In the corner...")
      self.integralArea_obstacle = 0  # reset
      self.previousError_obstacle = 0  # reset
      self.Kd_obstacle = 2.5  # change value of Kd to higher value
      self.Kp_obstacle = 9.0  # same for Kp - we want the robot to take the corner sharply, as closest as possilbe
      self.outer_corner = 1  # change value to "in the corner"
    if (self.robot.range[2].distance() < 1.0) and (self.outer_corner == 1):  # check if robot is leaving the corner
      print("Out of corner...")
      self.integralArea_obstacle = 0  # reset
      self.previousError_obstacle = 0  # reset
      self.Kp_obstacle = 1.2  # restore the default value
      self.Ki_obstacle = 0.0  # restore the default value
      self.Kd_obstacle = 0.2  # restore the default value
      self.outer_corner = 0  # change value to "out of the corner"
    if min([s.distance() for s in self.robot.range["left-front"]]) <= 0.3:  #check if the robot isnt too close to the obstacle
      self.robot.move(0.0, -0.5)
    error = self.setpoint - min([s.distance() for s in self.robot.range["left-front"]])  # calculate the actual error between the desired distance and actual distance
    self.integralArea_obstacle += error  # integral part of PID control
    correction = (self.Kp_obstacle * error + self.Ki_obstacle * self.integralArea_obstacle + self.Kd_obstacle * (
                self.previousError_obstacle - error))  # calculation of PID contol
    self.previousError_obstacle = error  # save the actual value of error in order to use it later
    if (correction < 0.1 and correction > -0.1):  # if the result of PID is very small, we use 0 instead
      correction = 0
    turn = min(max(-1.0, 0.0 - correction), 1.0)  # calculate turn ratio (value can be between -1, 1)
    speed = max(min(1.0, 1.0 - abs(correction)), 0.0)  # calculate speed ratio (value can be between -1, 1)
    self.robot.move(speed, turn)


  def follow_line(self):    #method which is used for following the line (using PID)
    print("Following line...")
    self.integralArea_line += self.errorLine
    correctionLine = (self.Kp_line * self.errorLine + self.Ki_line * self.integralArea_line + self.Kd_line * (
              self.previousError_line - self.errorLine))
    self.previousError_line = self.errorLine
    if (correctionLine < 0.1 and correctionLine > -0.1):
      correctionLine = 0
    turn = min(max(-1.0, 0.0 - correctionLine), 1.0)
    speed = max(min(1.0, 1.0 - abs(correctionLine)), 0.0)
    self.robot.move(speed, turn)


  def step(self):
    #get image and robots deviation
    self.get_image()
    if not self.modeWall:     #if robot is not following the obstacle, check if there is some
      self.check_wall()

    if self.modeWall:     #check if there is an obstacle ahead
      self.avoid_obstacle()
    elif self.foundLine and not self.modeWall:     #if there is a line but no obstacle
      self.follow_line()
      self.outer_corner = 0
    elif not self.foundLine and not self.modeWall:   #if there is no line and no obstacle
      print("Finding...")
      self.robot.move(1.0, 0.0)  # full speed ahead, just go until finding something

def INIT(engine):
  assert (engine.robot.requires("range-sensor") and
	  engine.robot.requires("continuous-movement"))

  return BrainFollowLine('BrainFollowLine', engine)
