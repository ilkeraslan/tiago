#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

foo = '/foo'

x = 0
y = 0
z = 0
yaw = 0

def pose_callback(res):
    global x
    global y
    x = res.x
    y = res.y
    yaw = res.theta
    rospy.logwarn(f'POSE CALLBACK X: {res.x}')


def degree2radian(deg):
    return (deg * 3.14159265359) / 180.0

def move(speed, distance, is_forward):
    velocity_msg = Twist()
    global x, y
    x0 = x
    y0 = y

    if (is_forward):
        velocity_msg.linear.x = abs(speed)
    else:
        velocity_msg.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = foo + '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Move forward")
        velocity_publisher.publish(velocity_msg)
        loop_rate.sleep()

        distance_moved = distance_moved + abs(0.5 * math.sqrt(((x - x0) ** 2)))
        if not (distance_moved < distance_moved):
            rospy.loginfo("Reached")
            break

        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)

def rotate(ang_speed_degree, relative_angle_degree, clockwise):
    global yaw
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    velocity_msg.linear.z = 0
    velocity_msg.angular.x = 0
    velocity_msg.angular.y = 0
    velocity_msg.angular.z = 0

    theta0 = yaw
    ang_speed = math.radians(abs(ang_speed_degree))

    if (clockwise):
        velocity_msg.angular.z = -abs(ang_speed)
    else:
        velocity_msg.angular.z = abs(ang_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = foo + '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Rotating")
        velocity_publisher.publish(velocity_msg)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1 - t0)*ang_speed_degree
        loop_rate.sleep()

        if(current_angle_degree > relative_angle_degree):
            rospy.loginfo("Reached")
            break

    velocity_msg.angular.z = 0
    velocity_publisher.publish(velocity_msg)

def go_to_goal(x_goal, y_goal):
    global x, y, z, yaw
    velocity_msg = Twist()
    cmd_vel_topic = foo + '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        K_linear = 0.5
        distance = abs(math.sqrt(((x_goao - x) ** 2) + ((y_goal - y) ** 2)))
        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal - y, x_goal - x)
        angular_speed = (desired_angle_goal - yaw) * K_angular

        velocity_msg.linear.x = linear_speed
        velocity_msg.angular.z = angular_speed
        velocity_publisher.publish(velocity_msg)

        if (distance < 0.01):
            break

def set_desired_orientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print(relative_angle_radians)
    print(desired_angle_radians)
    rotate(30, math.degrees(abs(relative_angle_radians)), clockwise)

def grid_clean():
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0

    move_goal(desired_pose, 0.01)
    set_desired_orientation(degrees2radians(desired_pose.theta))

    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 1.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(1.0, 9.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(2.0, 9.0, True)
    pass

def spiral_clean():
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    wk = 4
    rk = 0

    while((currentTurtlesimPose.x < 10.5) and currentTurtlesimPose.y < 10.5):
        rk = rk + 1
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = wk
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('tracker', anonymous=True)

        cmd_vel_topic = foo + '/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        position_topic = foo + '/pose'
        pose_subscriber = rospy.Subscriber(position_topic, Pose, pose_callback)
        time.sleep(2)
        
    except rospy.ROSInterruptException:
        pass
