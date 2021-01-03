#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from webots_ros.srv import get_float, set_int
from webots_ros.msg import Float64Stamped
from sensor_msgs.msg import Range

foo = '/foo'

def callback_proximity(res):
    rospy.loginfo(f'Proximity value: {res.range}')


def setup_proximity_sensors():
    # Wait for services
    rospy.wait_for_service(foo + "/prox_horizontal_1/enable")
    rospy.wait_for_service(foo + "/prox_horizontal_1/get_max_value")
    rospy.wait_for_service(foo + "/prox_horizontal_1/get_min_value")

    # Create services
    service_enable_horizontal_prox_1 = rospy.ServiceProxy(foo + "/prox_horizontal_1/enable", set_int)
    service_min_value_horizontal_prox_1 = rospy.ServiceProxy(foo + "/prox_horizontal_1/get_min_value", get_float)
    service_max_value_horizontal_prox_1 = rospy.ServiceProxy(foo + "/prox_horizontal_1/get_max_value", get_float)

    # Call services
    service_enable_horizontal_prox_1.call(10)
    service_min_value_horizontal_prox_1.call()
    service_max_value_horizontal_prox_1.call()

    # Subscribe to horizontal proximity 1 value
    horizontal_prox_1_value = rospy.Subscriber(foo + "/prox_horizontal_1/value", Range, callback_proximity)

    rospy.spin()


def talker():
    pub = rospy.Publisher('my_custom_topic', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        rospy.loginfo(foo)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        setup_proximity_sensors()
        # talker()
    except rospy.ROSInterruptException:
        pass
