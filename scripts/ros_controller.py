#!/usr/bin/env python

"""
This is a simple example of a Python ROS node receiving sensor values and publishing motor commands (velocity)
to drive a robot and stop it before colliding with an obstacle.
"""

import rospy
from std_msgs.msg import Float64
from webots_ros.srv import set_float, get_float, set_int
from webots_ros.msg import Float64Stamped
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# initialize the node if we want to use
rospy.init_node('controller', anonymous=True)

global foo
foo = '/foo'

global position
position = 0

def callback(res):
    # rospy.logwarn(f'Motor position: {res}')

    service_set_motor_velocity_left.call(res.linear.x)
    service_set_motor_velocity_right.call(res.linear.x)

    left_velocity = service_get_motor_velocity_left.call()
    right_velocity = service_get_motor_velocity_right.call()

    rospy.logwarn(f'Left velocity: {left_velocity}')
    rospy.logwarn(f'Right velocity: {right_velocity}')

    global position
    position += res.linear.x

    service_set_motor_position_left.call(position)
    service_set_motor_position_right.call(position)

    rospy.logwarn(f'Motor position: {position}')

    pose_message = Pose()
    pose_message.x = position
    pose_message.y = 0
    pose_publisher = rospy.Publisher(foo + "/pose", Pose, queue_size=10)
    pose_publisher.publish(pose_message)

# check for services
rospy.wait_for_service(foo + "/motor_left/set_position")
rospy.wait_for_service(foo + "/motor_right/set_position")

rospy.wait_for_service(foo + "/motor_left_position/enable")
rospy.wait_for_service(foo + "/motor_right_position/enable")

rospy.wait_for_service(foo + "/motor_left/set_velocity")
rospy.wait_for_service(foo + "/motor_right/set_velocity")

rospy.wait_for_service(foo + "/motor_left/get_velocity")
rospy.wait_for_service(foo + "/motor_right/get_velocity")

# create services
service_enable_motor_position_left = rospy.ServiceProxy(foo + "/motor_left_position/enable", set_int)
service_enable_motor_position_right = rospy.ServiceProxy(foo + "/motor_right_position/enable", set_int)

service_set_motor_position_left = rospy.ServiceProxy(foo + "/motor_left/set_position", set_float)
service_set_motor_position_right = rospy.ServiceProxy(foo + "/motor_right/set_position", set_float)

service_set_motor_velocity_left = rospy.ServiceProxy(foo + "/motor_left/set_velocity", set_float)
service_set_motor_velocity_right = rospy.ServiceProxy(foo + "/motor_right/set_velocity", set_float)

service_get_motor_velocity_left = rospy.ServiceProxy(foo + "/motor_left/get_velocity", get_float)
service_get_motor_velocity_right = rospy.ServiceProxy(foo + "/motor_right/get_velocity", get_float)

# # call set position services with 25 milliseconds sampling rate
# service_set_motor_position_left.call(100)
# service_set_motor_position_right.call(100)

# call enable position services with 16 milliseconds sampling rate
service_enable_motor_position_left.call(16)
service_enable_motor_position_right.call(16)

# # pub = rospy.Publisher('motor', Float64, queue_size=10)
# # Subscribe to motor position values
# motor_left_position = rospy.Subscriber(foo + "/motor_left_position/value", Float64Stamped, callback)
# motor_right_position = rospy.Subscriber(foo + "/motor_right_position/value", Float64Stamped, callback)

# Subscribe to motor position values
motor_left_position = rospy.Subscriber("/cmd_vel", Twist, callback)

rospy.spin()
