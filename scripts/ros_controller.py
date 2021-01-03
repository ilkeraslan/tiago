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
    rospy.logwarn(f'Motor position: {res.data}')

# check for services
rospy.wait_for_service(foo + "/motor_left/set_position")
rospy.wait_for_service(foo + "/motor_right/set_position")

rospy.wait_for_service(foo + "/motor_left_position/enable")
rospy.wait_for_service(foo + "/motor_right_position/enable")

# create services
service_enable_motor_position_left = rospy.ServiceProxy(foo + "/motor_left_position/enable", set_int)
service_enable_motor_position_right = rospy.ServiceProxy(foo + "/motor_right_position/enable", set_int)

service_set_motor_position_left = rospy.ServiceProxy(foo + "/motor_left/set_position", set_float)
service_set_motor_position_right = rospy.ServiceProxy(foo + "/motor_right/set_position", set_float)

# call set position services with 25 milliseconds sampling rate
service_set_motor_position_left.call(25)
service_set_motor_position_right.call(25)

# call enable position services with 16 milliseconds sampling rate
service_enable_motor_position_left.call(16)
service_enable_motor_position_right.call(16)

# pub = rospy.Publisher('motor', Float64, queue_size=10)
# Subscribe to motor position values
motor_left_position = rospy.Subscriber(foo + "/motor_left_position/value", Float64Stamped, callback)
motor_right_position = rospy.Subscriber(foo + "/motor_right_position/value", Float64Stamped, callback)

rospy.spin()
