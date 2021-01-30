#!/usr/bin/env python

"""
This is controller node receiving sensor values and publishing motor commands (velocity)
to drive Tiago and stop it before colliding with an obstacle.
"""

import rospy
import math
from std_msgs.msg import Float64
from webots_ros.srv import set_float, get_float, set_int, get_int
from webots_ros.msg import Float64Stamped, BoolStamped
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from sensor_msgs.msg import Range, LaserScan


class TiagoController:
    def position_callback_left(self, res):
        self.left_position = res.data

    def position_callback_right(self, res):
        self.right_position = res.data 

    def __init__(self, node_name):
        rospy.init_node('controller', anonymous=True)

        # self.name = rospy.get_param('~robot_name')
        self.name = "/foo"
        rospy.loginfo('Controlling %s' % self.name)

        self.check_for_services()
        self.create_services()
        self.call_services()

        self.velocity_publisher = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)
        self.bumper_subscriber = rospy.Subscriber(self.name + "/base_cover_link/value", BoolStamped, self.bumper_callback)
        self.left_position_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_left_joint_sensor/value', Float64Stamped, self.position_callback_left)
        self.right_position_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_right_joint_sensor/value', Float64Stamped, self.position_callback_right)

        self.left_velocity = 1
        self.right_velocity = 1
        self.left_position = 0
        self.right_position = 0
        
        self.move(3)

        rospy.spin()


    def bumper_callback(self, res):
        pass
        # rospy.logwarn(f'Bumped: {res.data}')
        # if (res.data is True):
        #     self.service_set_motor_velocity_left.call(1)
        #     self.service_set_motor_velocity_right.call(1)

    def move(self, distance):
        vel_msg = Twist()
        vel_msg.linear.x = (self.left_velocity + self.right_velocity) / 2
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        
        rotation = distance / (math.pi * 0.2)
        angle = rotation * 2 * math.pi
        left = self.left_velocity + angle
        right = self.right_velocity + angle

        while(current_distance < ((left + right)) / 2):

            self.service_set_motor_velocity_left.call(self.left_velocity)
            self.service_set_motor_velocity_right.call(self.right_velocity)

            self.service_set_motor_position_left.call(left)
            self.service_set_motor_position_right.call(right)
            
            #Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            
            #Calculates distancePoseStamped
            current_distance = ((self.left_velocity + self.right_velocity) / 2) * (t1-t0)
            rospy.logerr(f'CURRENT DISTANCE: {current_distance}')
        
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        self.service_set_motor_velocity_left.call(self.left_velocity)
        self.service_set_motor_velocity_right.call(self.right_velocity)
        
        #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)


    def check_for_services(self):
        rospy.wait_for_service(self.name + "/accelerometer/enable")
        rospy.wait_for_service(self.name + "/gyro/enable")
        rospy.wait_for_service(self.name + "/inertial_unit/enable")
        rospy.wait_for_service(self.name + "/Hokuyo_URG_04LX_UG01/enable")
        rospy.wait_for_service(self.name + "/base_cover_link/enable")

        rospy.wait_for_service(self.name + "/wheel_left_joint/set_velocity")
        rospy.wait_for_service(self.name + "/wheel_right_joint/set_velocity")

        rospy.wait_for_service(self.name + "/wheel_left_joint/set_position")
        rospy.wait_for_service(self.name + "/wheel_right_joint/set_position")

        rospy.wait_for_service(self.name + "/wheel_right_joint_sensor/enable")
        rospy.wait_for_service(self.name + "/wheel_left_joint_sensor/enable")


    def create_services(self):
        self.service_set_motor_velocity_left = rospy.ServiceProxy(self.name + "/wheel_left_joint/set_velocity", set_float)
        self.service_set_motor_velocity_right = rospy.ServiceProxy(self.name + "/wheel_right_joint/set_velocity", set_float)

        self.service_enable_motor_position_left = rospy.ServiceProxy(self.name + "/wheel_left_joint_sensor/enable", set_int)
        self.service_enable_motor_position_right = rospy.ServiceProxy(self.name + "/wheel_right_joint_sensor/enable", set_int)

        self.service_set_motor_position_left = rospy.ServiceProxy(self.name + "/wheel_left_joint/set_position", set_float)
        self.service_set_motor_position_right = rospy.ServiceProxy(self.name + "/wheel_right_joint/set_position", set_float)

        self.service_accelerometer = rospy.ServiceProxy(self.name + "/accelerometer/enable", set_int)
        self.service_gyro = rospy.ServiceProxy(self.name + "/gyro/enable", set_int)
        self.service_inertial = rospy.ServiceProxy(self.name + "/inertial_unit/enable", set_int)
        self.service_lidar = rospy.ServiceProxy(self.name + "/Hokuyo_URG_04LX_UG01/enable", set_int)
        self.service_touch_sensor = rospy.ServiceProxy(self.name + "/base_cover_link/enable", set_int)


    def call_services(self):
        self.service_accelerometer.call(10)
        self.service_gyro.call(10)
        self.service_inertial.call(10)
        self.service_lidar.call(10)
        self.service_touch_sensor.call(10)
        self.service_enable_motor_position_left.call(10)
        self.service_enable_motor_position_right.call(10)
        
        # service_set_motor_position_left.call(float('+inf'))
        # service_set_motor_position_right.call(float('+inf'))

        # service_set_motor_velocity_left.call(3)
        # service_set_motor_velocity_right.call(3)


if __name__ == '__main__':
    try:
        TiagoController("/foo")
    except rospy.ROSInterruptException: 
        pass



# TODO: rotation
# TODO: odometry update
# TODO: Astar


# def callback(res):
#     # rospy.logwarn(f'Motor position: {res}')

#     service_set_motor_velocity_left.call(res.linear.x)
#     service_set_motor_velocity_right.call(res.linear.x)

#     left_velocity = service_get_motor_velocity_left.call()
#     right_velocity = service_get_motor_velocity_right.call()

#     rospy.logwarn(f'Left velocity: {left_velocity}')
#     rospy.logwarn(f'Right velocity: {right_velocity}')

#     service_set_motor_position_left.call(position)
#     service_set_motor_position_right.call(position)

#     rospy.logerr(f'Motor position: {position}')

#     pose_message = Pose()
#     pose_message.x = position
#     pose_message.y = 0
#     pose_publisher = rospy.Publisher(foo + "/pose", Pose, queue_size=10)
#     pose_publisher.publish(pose_message)

