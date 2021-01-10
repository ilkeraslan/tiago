#!/usr/bin/env python

import math

import rospy
import tf2_ros
from std_msgs.msg import Time
from webots_ros.srv import set_float, get_float, set_int
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Range, LaserScan

foo = '/foo'
scan_publisher_0 = None
scan_publisher_1 = None
scan_publisher_2 = None
scan_publisher_3 = None
scan_publisher_4 = None
scan_publisher_5 = None
scan_publisher_6 = None

def callback(value):
    rospy.logwarn(f'{value.header.frame_id}: {value.range}')
    
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = value.header.frame_id
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14
    scan.time_increment = (1 / 40) / (100)
    scan.range_min = value.min_range
    scan.range_max = value.max_range

    scan.ranges = []
    scan.intensities = []
    for i in range(0, 100):
        scan.ranges.append(value.range)

    if value.header.frame_id == 'foo/prox_horizontal_0':
        scan_publisher_0.publish(scan)
    elif value.header.frame_id == 'foo/prox_horizontal_1':
        scan_publisher_1.publish(scan)
    elif value.header.frame_id == 'foo/prox_horizontal_2':
        scan_publisher_2.publish(scan)
    elif value.header.frame_id == 'foo/prox_horizontal_3':
        scan_publisher_3.publish(scan)
    elif value.header.frame_id == 'foo/prox_horizontal_4':
        scan_publisher_4.publish(scan)
    elif value.header.frame_id == 'foo/prox_horizontal_5':
        scan_publisher_5.publish(scan)
    else:
        scan_publisher_6.publish(scan)


def enable_scanners():
    rospy.wait_for_service(foo + "/prox_horizontal_0/enable")
    rospy.wait_for_service(foo + "/prox_horizontal_1/enable")
    rospy.wait_for_service(foo + "/prox_horizontal_2/enable")
    rospy.wait_for_service(foo + "/prox_horizontal_3/enable")
    rospy.wait_for_service(foo + "/prox_horizontal_4/enable")
    rospy.wait_for_service(foo + "/prox_horizontal_5/enable")
    rospy.wait_for_service(foo + "/prox_horizontal_6/enable")

    service_enable_horizontal_0 = rospy.ServiceProxy(foo + "/prox_horizontal_0/enable", set_int)
    service_enable_horizontal_1 = rospy.ServiceProxy(foo + "/prox_horizontal_1/enable", set_int)
    service_enable_horizontal_2 = rospy.ServiceProxy(foo + "/prox_horizontal_2/enable", set_int)
    service_enable_horizontal_3 = rospy.ServiceProxy(foo + "/prox_horizontal_3/enable", set_int)
    service_enable_horizontal_4 = rospy.ServiceProxy(foo + "/prox_horizontal_4/enable", set_int)
    service_enable_horizontal_5 = rospy.ServiceProxy(foo + "/prox_horizontal_5/enable", set_int)
    service_enable_horizontal_6 = rospy.ServiceProxy(foo + "/prox_horizontal_6/enable", set_int)

    service_enable_horizontal_0.call(10)
    service_enable_horizontal_1.call(10)
    service_enable_horizontal_2.call(10)
    service_enable_horizontal_3.call(10)
    service_enable_horizontal_4.call(10)
    service_enable_horizontal_5.call(10)
    service_enable_horizontal_6.call(10)

    rospy.Subscriber(foo + '/prox_horizontal_0/value', Range, callback)
    rospy.Subscriber(foo + '/prox_horizontal_1/value', Range, callback)
    rospy.Subscriber(foo + '/prox_horizontal_2/value', Range, callback)
    rospy.Subscriber(foo + '/prox_horizontal_3/value', Range, callback)
    rospy.Subscriber(foo + '/prox_horizontal_4/value', Range, callback)
    rospy.Subscriber(foo + '/prox_horizontal_5/value', Range, callback)
    rospy.Subscriber(foo + '/prox_horizontal_6/value', Range, callback)

if __name__ == '__main__':
    rospy.init_node('scanners')

    scan_publisher_0 = rospy.Publisher(foo + '/scan_0', LaserScan, queue_size=50)
    scan_publisher_1 = rospy.Publisher(foo + '/scan_1', LaserScan, queue_size=50)
    scan_publisher_2 = rospy.Publisher(foo + '/scan_2', LaserScan, queue_size=50)
    scan_publisher_3 = rospy.Publisher(foo + '/scan_3', LaserScan, queue_size=50)
    scan_publisher_4 = rospy.Publisher(foo + '/scan_4', LaserScan, queue_size=50)
    scan_publisher_5 = rospy.Publisher(foo + '/scan_5', LaserScan, queue_size=50)
    scan_publisher_6 = rospy.Publisher(foo + '/scan_6', LaserScan, queue_size=50)

    rospy.sleep(rospy.Duration(2))

    enable_scanners()

    rospy.spin()
