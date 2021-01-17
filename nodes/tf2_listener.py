#!/usr/bin/env python  

import rospy
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    robot_name = rospy.get_param('~robot_name')
    publisher = rospy.Publisher('%s/cmd_vel' % robot_name, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("world", "prox_horizontal_0", rospy.Time())
            rospy.logerr("RECEIVED TRANSFORM")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            rospy.logerr("EXCEPTION")
            continue

        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        msg.linear.x = 3

        publisher.publish(msg)

        rate.sleep()
