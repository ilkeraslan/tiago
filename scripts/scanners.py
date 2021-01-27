#!/usr/bin/env python

import math

import rospy
import tf2_ros
from std_msgs.msg import Time
from webots_ros.srv import set_float, get_float, set_int
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import tf_conversions
import tf2_msgs.msg

foo = '/foo'
scan_publisher = None
front_distance = None

def callback(value):
    # rospy.logwarn(f'{value.header.frame_id}: {value.ranges}')
    # rospy.logwarn(f'{value.header.frame_id}: {value.ranges}')
    # value.ranges // 667 elements
    front_distance = min(value.ranges[278:388])

    if(front_distance < 0.4):
        rospy.logerr("OBSTACLE")
    

def publish_frame(child_frame_id, range_value):
    br = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = child_frame_id

    t.transform.translation.x = range_value
    t.transform.translation.y = range_value
    t.transform.translation.z = 0.0

    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    tfm = tf2_msgs.msg.TFMessage([t])
    br.publish(tfm)


def enable_scanners():
    rospy.Subscriber(foo + '/Hokuyo_URG_04LX_UG01/laser_scan/layer0', LaserScan, callback)


if __name__ == '__main__':
    rospy.init_node('scanners')

    scan_publisher_0 = rospy.Publisher(foo + '/lidar_scan', LaserScan, queue_size=10)

    rospy.sleep(rospy.Duration(2))

    enable_scanners()

    rospy.spin()
