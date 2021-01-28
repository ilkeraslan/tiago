#!/usr/bin/env python

"""
This is a simple example of a Python ROS node receiving sensor values and publishing motor commands (velocity)
to drive a robot and stop it before colliding with an obstacle.
"""

import rospy
import math
from std_msgs.msg import Float64
from webots_ros.srv import set_float, get_float, set_int, get_int
from webots_ros.msg import Float64Stamped, BoolStamped
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Range, LaserScan


bumper_subscriber = None
left_vel_sensor_subscriber = None
right_vel_sensor_subscriber = None
service_accelerometer = None
service_gyro = None
service_inertial = None
service_lidar = None
service_touch_sensor = None

left_velocity = 0.0
right_velocity = 0.0

class TiagoController:
    def __init__(self, node_name):
        rospy.init_node('controller', anonymous=True)

        # self.name = rospy.get_param('~robot_name')
        self.name = "/foo"
        rospy.loginfo('Controlling %s' % self.name)

        self.velocity_publisher = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)

        self.bumper_subscriber = rospy.Subscriber(self.name + "/base_cover_link/value", BoolStamped, self.bumper_callback)
        self.left_vel_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_left_joint_sensor/value', Float64Stamped, self.velocity_callback_left)
        self.right_vel_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_right_joint_sensor/value', Float64Stamped, self.velocity_callback_right)

        self.check_for_services()
        self.create_services()
        self.call_services()

        self.move(0.7)
        rospy.sleep(rospy.Duration(5))
        self.move(-0.7)

        rospy.spin()


    def bumper_callback(self, res):
        # rospy.logwarn(f'Bumped: {res.data}')
        if (res.data is True):
            service_set_motor_velocity_left.call(-3)
            service_set_motor_velocity_right.call(-3)


    def velocity_callback_left(self, res):
        global left_velocity
        left_velocity = res.data 


    def velocity_callback_right(self, res):
        global right_velocity
        right_velocity = res.data 



    def move(self, distance):
        global left_velocity
        global right_velocity

        rotation = distance / (math.pi * 0.2)
        angle = rotation * 2 * math.pi
        left = left_velocity + angle
        right = right_velocity + angle

        self.service_set_motor_position_left.call(left)
        self.service_set_motor_position_right.call(right)

        self.service_set_motor_velocity_left.call(3)
        self.service_set_motor_velocity_right.call(3)
        self.stop()

    def stop(self):
        global left_velocity
        global right_velocity
        while (left_velocity > 0.1 and right_velocity > 0.1):
            pass

    def check_for_services(self):
        rospy.wait_for_service(self.name + "/accelerometer/enable")
        rospy.wait_for_service(self.name + "/gyro/enable")
        rospy.wait_for_service(self.name + "/inertial_unit/enable")
        rospy.wait_for_service(self.name + "/Hokuyo_URG_04LX_UG01/enable")
        rospy.wait_for_service(self.name + "/base_cover_link/enable")

        rospy.wait_for_service(self.name + "/wheel_left_joint/set_velocity")
        rospy.wait_for_service(self.name + "/wheel_right_joint/set_velocity")

        rospy.wait_for_service(self.name + "/wheel_left_joint/set_position")
        rospy.wait_for_service(self.name + "/wheel_right_joint/set_position")

        rospy.wait_for_service(self.name + "/wheel_right_joint_sensor/enable")
        rospy.wait_for_service(self.name + "/wheel_left_joint_sensor/enable")


    def create_services(self):
        self.service_set_motor_velocity_left = rospy.ServiceProxy(self.name + "/wheel_left_joint/set_velocity", set_float)
        self.service_set_motor_velocity_right = rospy.ServiceProxy(self.name + "/wheel_right_joint/set_velocity", set_float)

        self.service_get_motor_velocity_left = rospy.ServiceProxy(self.name + "/wheel_left_joint_sensor/enable", set_int)
        self.service_get_motor_velocity_right = rospy.ServiceProxy(self.name + "/wheel_right_joint_sensor/enable", set_int)

        self.service_set_motor_position_left = rospy.ServiceProxy(self.name + "/wheel_left_joint/set_position", set_float)
        self.service_set_motor_position_right = rospy.ServiceProxy(self.name + "/wheel_right_joint/set_position", set_float)

        self.service_accelerometer = rospy.ServiceProxy(self.name + "/accelerometer/enable", set_int)
        self.service_gyro = rospy.ServiceProxy(self.name + "/gyro/enable", set_int)
        self.service_inertial = rospy.ServiceProxy(self.name + "/inertial_unit/enable", set_int)
        self.service_lidar = rospy.ServiceProxy(self.name + "/Hokuyo_URG_04LX_UG01/enable", set_int)
        self.service_touch_sensor = rospy.ServiceProxy(self.name + "/base_cover_link/enable", set_int)


    def call_services(self):
        self.service_accelerometer.call(10)
        self.service_gyro.call(10)
        self.service_inertial.call(10)
        self.service_lidar.call(10)
        self.service_touch_sensor.call(10)
        self.service_get_motor_velocity_left.call(10)
        self.service_get_motor_velocity_right.call(10)
        
        # service_set_motor_position_left.call(float('+inf'))
        # service_set_motor_position_right.call(float('+inf'))

        # service_set_motor_velocity_left.call(3)
        # service_set_motor_velocity_right.call(3)

TiagoController("/foo")


# TODO: rotation
# TODO: odometry update
# TODO: Astar


# def callback(res):
#     # rospy.logwarn(f'Motor position: {res}')

#     service_set_motor_velocity_left.call(res.linear.x)
#     service_set_motor_velocity_right.call(res.linear.x)

#     left_velocity = service_get_motor_velocity_left.call()
#     right_velocity = service_get_motor_velocity_right.call()

#     rospy.logwarn(f'Left velocity: {left_velocity}')
#     rospy.logwarn(f'Right velocity: {right_velocity}')

#     service_set_motor_position_left.call(position)
#     service_set_motor_position_right.call(position)

#     rospy.logerr(f'Motor position: {position}')

#     pose_message = Pose()
#     pose_message.x = position
#     pose_message.y = 0
#     pose_publisher = rospy.Publisher(foo + "/pose", Pose, queue_size=10)
#     pose_publisher.publish(pose_message)

