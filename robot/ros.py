"""
Defines RosRobot, a subclass of robot

(c) 2005, PyrobRobotics.org. Licenced under the GNU GPL.
"""

import time, math
from pyrobot.robot import *
from pyrobot.robot.device import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import rospy
import tf

class RosSonarDevice(Device):
    def __init__(self, robot):
        self.robot = robot
        Device.__init__(self)  # "sonar", visible=visible

        # Subscribe to the robot's sensor
        if (self.robot.robotType == RosRobot.STAGE):
          rospy.Subscriber("base_scan", LaserScan, self.callbackSonar)
          # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html
          self.datasonar = LaserScan()
        else:
          rospy.Subscriber("RosAria/sonar", PointCloud, self.callbackSonar)
          # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud.html
          self.datasonar = PointCloud()

        # Wait until we get an initial reading??
        while len(self) == 0:
            pass
        if len(self) == 8:
            # this data should come from the robot and should not be
            # hard coded here
            self.poseSensor = [
                [1.57, 0.87, 0.52, 0.17, -0.17, -0.52, -0.87, -1.75],  # rads
                [
                    -0.013,
                    -0.0115,
                    -0.0075,
                    -0.002,
                    0.002,
                    0.0075,
                    0.0115,
                    0.013,
                ],  # x in meters
                [0.007, 0.0115, 0.0145, 0.0165, 0.0165, 0.0145, 0.00115, 0.007],
            ]  # y
            self.groups = {
                "all": list(range(8)),
                "front": (3, 4),
                "front-left": (1, 2, 3),
                "front-right": (4, 5, 6),
                "front-all": (1, 2, 3, 4, 5, 6),
                "front-to-back-left": (3, 2, 1, 0),  # in order
                "front-to-back-right": (4, 5, 6, 7),  # in order
                "left": (0,),
                "right": (7,),
                "left-front": (0,),
                "right-front": (7,),
                "left-back": (),
                "right-back": (),
                "back-right": (),
                "back-left": (),
                "back": (),
                "back-all": (),
            }
        elif len(self) == 16:
            self.groups = {
                "all": list(range(16)),
                "front": (3, 4),
                "front-left": (1, 2, 3),
                "front-right": (4, 5, 6),
                "front-all": (1, 2, 3, 4, 5, 6),
                "front-to-back-left": (3, 2, 1, 0, 15, 14, 13, 12),  # in order
                "front-to-back-right": (4, 5, 6, 7, 8, 9, 10, 11),  # in order
                "left": (0, 15),
                "right": (7, 8),
                "left-front": (0,),
                "right-front": (7,),
                "left-back": (15,),
                "right-back": (8,),
                "back-right": (9, 10, 11),
                "back-left": (12, 13, 14),
                "back": (11, 12),
                "back-all": (9, 10, 11, 12, 13, 14),
            }
        else:
            self.groups = {"all": list(range(len(self)))}

        self.units = "METERS"
        # Anything that you pass to rawToUnits should be in these units
        self.rawunits = "METERS"
        # get this from the sensor when possible
        if (self.robot.robotType == RosRobot.STAGE):
          self.maxvalueraw = self.datasonar.range_max
        else:
          self.maxvalueraw = 5.0  # meters
        # These are fixed in meters: DO NOT CONVERT ----------------
        self.radius = 0.40  # meters
        # ----------------------------------------------------------
        # All of the rest of the measures are relative to units, given
        # in rawunits:
        self.count = len(self)
        self._noise = 0.05 # 5% noise
        self.arc = 7.5 * PIOVER180

    # funcion llamada por la recepcion de los datos de la cola.
    def callbackSonar(self, data):
        self.datasonar = data

    def __len__(self):
        if (self.robot.robotType == RosRobot.STAGE):
          return len(self.datasonar.ranges)
        return 8  # FIXED, should come from ROS somehow!

    def getSensorValue(self, pos):
        if pos < 16:
            z = 0.23
        else:
            z = 1.1
        if (self.robot.robotType == RosRobot.STAGE):
          scan = self.datasonar.ranges[(len(self)-1)-pos]
        else: 
          x = self.datasonar.points[pos].x
          y = self.datasonar.points[pos].y
          scan = math.sqrt((x * x) + (y * y)) - 0.5

        return SensorValue(
            self,
            scan,
            pos, 
            (
                self.poseSensor[1][pos],  # x in meters
                self.poseSensor[2][pos],  # y
                z,  # z
                self.poseSensor[0][pos],  # rads
                self.arc,
            ),
            noise= self._noise,
        )

class RosPositionDevice(Device):
    def __init__(self, robot):
        self.robot = robot
        Device.__init__(self)  # , "position", visible=visible

        # Subscribe to the robot's sensor
        if (self.robot.robotType == RosRobot.STAGE):
          rospy.Subscriber("odom", Odometry, self.callbackPosition)
        else:
          rospy.Subscriber("RosAria/pose", Odometry, self.callbackPosition)

    def addWidgets(self, window):
        window.addData("x", ".x:", self.robot.dataPose.pose.pose.position.x)
        window.addData("y", ".y:", self.robot.dataPose.pose.pose.position.y)
        window.addData("thr", ".th (angle in radians):", self.robot.radian)
        window.addData("th", ".thr (angle in degrees):", self.robot.radian / PIOVER180)
        # window.addData("stall", ".stall:", self._dev.stall)

    def updateWindow(self):
        if self.visible:
            self.window.updateWidget("x", self.robot.dataPose.pose.pose.position.x)
            self.window.updateWidget("y", self.robot.dataPose.pose.pose.position.y)
            self.window.updateWidget("thr", self.robot.radian)
            self.window.updateWidget("th", self.robot.radian / PIOVER180)
        # self.window.updateWidget("stall",self._dev.stall)

    def callbackPosition(self, data):
        self.robot.dataPose = data
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.robot.radian = euler[2]

class RosBatteryDevice(Device):
    def __init__(self,robot):

        self.robot=robot
        Device.__init__(self)

        # Subscribe to the robot's sensor
        if (self.robot.robotType == RosRobot.STAGE):
          pass # we aren't worried about the battery when using a sim
        else:
          rospy.Subscriber("RosAria/battery_voltage", Float64, self.callbackBattery)
        self.dataBattery = Float64()

    def addWidgets(self, window):
        window.addData("Battery", ".battery:", self.dataBattery.data)

    def updateWindow(self):
        if self.visible:
            self.window.updateWidget("Battery", self.dataBattery.data)

    def callbackBattery(self, data):
        self.dataBattery = data

class RosRobot(Robot):
    ROSARIA = 1
    STAGE = 2

    def __init__(self, robotType,
                 startDevices=1, rosDevices=["battery"]):
        Robot.__init__(self)  # robot constructor
        self.robotType = robotType

        if (robotType == RosRobot.STAGE):
          self.scaleVelocity = 0.7
        else:
          self.scaleVelocity = 0.2

        self.last_rotate = 0.0
        self.last_translate = 0.0
        # self.simulated = 1 

        if (robotType == RosRobot.STAGE):
          self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        else:
          self.pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=1000)

        rospy.init_node("pyrobot", anonymous=True)
        self.msg = Twist()
        self.dataPose = Odometry()
        self.radian = 0

        # a robot with no devices will hang here!

        # default values for all robots:
        self.stall = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.thr = 0.0
        # Can we get these from Ros?
        self.radius = 0.40
        self.type = "Ros"
        self.subtype = 0
        self.units = "METERS"
        self.localize(0.0, 0.0, 0.0)
        self.update()
        self.builtinDevices = rosDevices
        # self.builtinDevices = ["sonar", "position", "battery"]
        if startDevices:
            for device in [
                "position",
                "laser",
                "ir",
                "sonar",
                "bumper",
                "speech",
                "graphics2d",
                "gripper",
                "battery",
            ]:

                # is it supported? if so start it up:
                if device in self.builtinDevices:

                    # try: # this is for gazebo; can't tell what it really has
                    deviceName = self.startDevice(device)
                    # except:
                    #    continue
                    # Make sure laser is before sonar, so if you have
                    # sonar, it will be the default 'range' device
                    if device == "laser":
                        self.range = self.laser[0]
                    elif device == "ir":
                        self.range = self.ir[0]
                    elif device == "sonar":
                        self.range = self.sonar[0]
                    elif device == "position":
                        self.supportedFeatures.append("odometry")
                        self.supportedFeatures.append("continuous-movement")
        if "range" in self.__dict__:
            self.supportedFeatures.append("range-sensor")

    def destroy(self):
        try:
            self.simulation.join()
        except:
            pass

    # Used to open an interface to an additional Ros device.  eg,
    # if you want to open position:10 in addition to position:0.
    #
    # name and idx correspond to the Ros interface type and device
    # number.
    #
    # Override is whether the new device should override devices
    # already open (eg, if some other position is being used by
    # default, should this override it or leave it alone).
    def startRosDevice(self, name, idx, override=False):
        d = self.robot.startDeviceBuiltin(name, idx)
        self.robot.startDevices(d, override=override)

    def startDeviceBuiltin(self, item, index=0):
        if item == "sonar":
            return {"sonar": RosSonarDevice(self)}
        elif item == "position":
            return {"position": RosPositionDevice(self)}
        elif item == "battery":
            return {"battery": RosBatteryDevice(self)}
        return {item: RosDevice(self._client, item, index=index)}

    def translate(self, translate_velocity):
        self.move(translate_velocity, self.last_rotate)

    def rotate(self, rotate_velocity):
        self.move(self.last_translate, rotate_velocity)

    def move(self, translate_velocity, rotate_velocity):
        tv = translate_velocity * self.scaleVelocity
        rv = rotate_velocity * self.scaleVelocity
        self.last_rotate = rv
        self.last_translate = tv

        self.msg.linear.x = tv
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0

        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = rv

        self.pub.publish(self.msg)
        time.sleep(0.1)

    def update(self):
        if self.hasA("position"):
            self.x = self.dataPose.pose.pose.position.x
            self.y = self.dataPose.pose.pose.position.y
            self.thr = self.radian  # radians
            self.th = self.thr / PIOVER180
            # self.stall = self.position[0]._dev.stall
        self.updateDevices()

    def disconnect(self):
        rospy.signal_shutdown("ROS Robot shutting down.")

    def localize(self, x=0, y=0, th=0):
        """
        Set robot's internal pose to x (meters), y (meters),
        th (radians)
        """
        if self.hasA("position"):
            # self._client.set_odometry(x * 1000, y * 1000, th)
            self.x = x
            self.y = y
            self.th = th
            self.thr = self.th * PIOVER180

    def removeDevice(self, item, number=0):
        self.__dict__[item][number]._dev.unsubscribe()
        Robot.removeDevice(self, item)


if __name__ == "__main__":
    myrobot = RosRobot()
    myrobot.update()
    myrobot.translate(0.2)
    myrobot.translate(0.0)
    myrobot.disconnect()
