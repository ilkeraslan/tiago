#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "world"
t.child_frame_id = "foo"

x = 0.0
y = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vth = 0.1

# vx = speed;
# vy = 0;
# vth = ((right_speed - left_speed)/lengthWheelBase);

current_time = rospy.Time.now()
last_time = rospy.Time.now()
r = rospy.Rate(1.0)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, th)

    t.transform.rotation.x = odom_quat[0]
    t.transform.rotation.y = odom_quat[1]
    t.transform.rotation.z = odom_quat[2]
    t.transform.rotation.w = odom_quat[3]

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(t)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
