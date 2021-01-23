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
from sensor_msgs.msg import Range, LaserScan

# initialize the node if we want to use
rospy.init_node('controller', anonymous=True)

global foo
foo = '/foo'

def callback(res):
    # rospy.logwarn(f'Motor position: {res}')

    service_set_motor_velocity_left.call(res.linear.x)
    service_set_motor_velocity_right.call(res.linear.x)

    left_velocity = service_get_motor_velocity_left.call()
    right_velocity = service_get_motor_velocity_right.call()

    rospy.logwarn(f'Left velocity: {left_velocity}')
    rospy.logwarn(f'Right velocity: {right_velocity}')

    service_set_motor_position_left.call(position)
    service_set_motor_position_right.call(position)

    rospy.logerr(f'Motor position: {position}')

    pose_message = Pose()
    pose_message.x = position
    pose_message.y = 0
    pose_publisher = rospy.Publisher(foo + "/pose", Pose, queue_size=10)
    pose_publisher.publish(pose_message)
  


# Check for services
rospy.wait_for_service(foo + "/accelerometer/enable")
rospy.wait_for_service(foo + "/gyro/enable")
rospy.wait_for_service(foo + "/inertial_unit/enable")
rospy.wait_for_service(foo + "/Hokuyo_URG_04LX_UG01/enable")

rospy.wait_for_service(foo + "/wheel_left_joint/set_velocity")
rospy.wait_for_service(foo + "/wheel_right_joint/set_velocity")
rospy.wait_for_service(foo + "/wheel_left_joint/set_position")
rospy.wait_for_service(foo + "/wheel_right_joint/set_position")


# Create services
service_set_motor_velocity_left = rospy.ServiceProxy(foo + "/wheel_left_joint/set_velocity", set_float)
service_set_motor_velocity_right = rospy.ServiceProxy(foo + "/wheel_right_joint/set_velocity", set_float)
service_set_motor_position_left = rospy.ServiceProxy(foo + "/wheel_left_joint/set_position", set_float)
service_set_motor_position_right = rospy.ServiceProxy(foo + "/wheel_right_joint/set_position", set_float)

service_lidar = rospy.ServiceProxy(foo + "/Hokuyo_URG_04LX_UG01/enable", set_int)

service_set_motor_position_left.call(float('+inf'))
service_set_motor_position_right.call(float('+inf'))
service_set_motor_velocity_left.call(3)
service_set_motor_velocity_right.call(3)
service_lidar.call(10)


# # Subscribe to motor position values
# motor_left_position = rospy.Subscriber(foo + "/cmd_vel", Twist, callback)

# scan_publisher_0 = rospy.Subscriber(foo + '/scan_0', LaserScan, my_callback_turn_left)
# scan_publisher_2 = rospy.Subscriber(foo + '/scan_2', LaserScan, my_callback)
# scan_publisher_4 = rospy.Subscriber(foo + '/scan_4', LaserScan, my_callback_turn_right)


rospy.spin()
