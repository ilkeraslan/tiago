#!/usr/bin/env python

"""
This is a simple example of a Python ROS node receiving sensor values and publishing motor commands (velocity)
to drive a robot and stop it before colliding with an obstacle.
"""

import rospy
from std_msgs.msg import Float64
from webots_ros.srv import set_float, set_int
from webots_ros.msg import Float64Stamped

# initialize the node if we want to use
rospy.init_node('controller', anonymous=True)

foo = '/foo'

def callback(res):
    rospy.logerr(str(res.data))

# check for the services
rospy.wait_for_service(foo + "/motor_left/set_position")
rospy.wait_for_service(foo + "/motor_right/set_position")

rospy.wait_for_service(foo + "/motor_left_position/enable")
rospy.wait_for_service(foo + "/motor_right_position/enable")


# create the service
motor_service_left = rospy.ServiceProxy(foo + "/motor_left/set_position", set_float)
motor_service_right = rospy.ServiceProxy(foo + "/motor_right/set_position", set_float)

motor_aaa_right = rospy.ServiceProxy(foo + "/motor_right_position/enable", set_int)
motor_bbb_right = rospy.ServiceProxy(foo + "/motor_left_position/enable", set_int)

# call the service
aaa = motor_service_left.call(25)
bbb = motor_service_right.call(25)

# Each 16 milliseconds is a sampling rate
ccc = motor_aaa_right.call(16)
ddd = motor_bbb_right.call(16)


# pub = rospy.Publisher('motor', Float64, queue_size=10)
ff = rospy.Subscriber(foo + "/motor_left_position/value", Float64Stamped, callback)

rospy.spin()
