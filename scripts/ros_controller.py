#!/usr/bin/env python

"""
This is controller node receiving sensor values and publishing motor commands (velocity)
to drive Tiago and stop it before colliding with an obstacle.
"""

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from webots_ros.srv import set_float, get_float, set_int, get_int
from webots_ros.msg import Float64Stamped, BoolStamped, Int32Stamped
from geometry_msgs.msg import Point, Twist, Vector3
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, LaserScan


class TiagoController:

    def position_callback_left(self, res):
        self.left_position = res.data

    def position_callback_right(self, res):
        self.right_position = res.data

    def callback(self,value):
        global isFree
        # value.ranges // 667 elements
        front_distance = min(value.ranges[278:388])

        if(front_distance < 0.3):
            isFree = False            
        else:    
            isFree = True

    
    def __init__(self, node_name):
        rospy.init_node('controller', anonymous=True)

        # self.name = rospy.get_param('~robot_name')
        self.name = "/foo"
        rospy.loginfo('Controlling %s' % self.name)

        self.check_for_services()
        self.create_services()
        self.call_services()

        self.left_velocity = 2
        self.right_velocity = 2
        self.left_position = 0
        self.right_position = 0
        self.orientation = 0 #0, 90, 180, 270
        self.scan_publisher = None
        self.front_distance = None
        self.isFree = True

        self.velocity_publisher = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)
        self.bumper_subscriber = rospy.Subscriber(self.name + "/base_cover_link/value", BoolStamped, self.bumper_callback)
        self.left_position_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_left_joint_sensor/value', Float64Stamped, self.position_callback_left) #distanza percorsa dalla ruota sx
        self.right_position_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_right_joint_sensor/value', Float64Stamped, self.position_callback_right)
        self.keyboard_subscriber = rospy.Subscriber(self.name + "/keyboard/key", Int32Stamped, self.keyboard_callback)
        self.scan_publisher_0 = rospy.Publisher(self.name + '/lidar_scan', LaserScan, queue_size=10)
        self.lidar_sub = rospy.Subscriber(self.name + '/Hokuyo_URG_04LX_UG01/laser_scan/layer0', LaserScan, self.callback)

        rospy.logerr(f'Left post: {self.left_position}')
        rospy.logerr(f'Right post: {self.right_position}')
        

        # self.rotate(90,True)
        # rospy.sleep(1)
        # self.move(65, 0)
        # rospy.sleep(1)
        # self.rotate(90,False)
        # rospy.sleep(1)
        # self.move(10, 0)
        # rospy.sleep(1)
        # self.rotate(90,False)
        # rospy.sleep(1)
        # self.move(19, 0)
        # rospy.sleep(1)
        # self.rotate(90,True)
        # rospy.sleep(1)
        # self.move(25, 0)
        # rospy.sleep(1)
        # self.move(25, 0)

        rospy.spin()

    def go_to_kitchen(self):
        self.rotate(180, False)
        rospy.sleep(1)
        self.move(12, 0)
        rospy.sleep(1)
        self.rotate(90, True)
        self.move(65, 0)
        rospy.sleep(1)
        self.rotate(90, False)
        rospy.sleep(1)

    def keyboard_callback(self, res):
        key=res.data
        if (key == 65):
            self.go_to_kitchen()
            self.move(19, 0)
            rospy.sleep(1)
            self.rotate(90, False)
            rospy.sleep(1)
            self.move(15, 270)
            rospy.sleep(1)
            self.rotate(90, True)
            rospy.sleep(1)
            self.move(17, 0)
            rospy.sleep(1)
        elif (key == 90):
            self.move_to_entrance_from_a()

    def move_to_a(self):
        self.move(110, 0)
        rospy.sleep(1)
        self.rotate(90)
        rospy.sleep(1)
        self.move(19, 90)
        rospy.sleep(1)
        self.rotate(270)
        rospy.sleep(1)
        self.move(20, 0)
        rospy.sleep(1)

    def move_to_entrance_from_a(self):
        self.rotate(180, False)
        rospy.sleep(1)
        self.move(22, 0)
        rospy.sleep(1)
        self.rotate(90, True)
        rospy.sleep(1)
        self.move(49, 90)
        rospy.sleep(1)
        self.rotate(90, False)
        rospy.sleep(1)
        self.move(28, 0)
        rospy.sleep(1)

    def move(self, distance, orientation):
        global x,y,yaw, isFree, isBumped, RobotPose
        x0=x
        y0=y
        delta_distance_x = 0
        delta_distance_y = 0
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        if (orientation == 0):
            vel_msg.linear.x = (self.left_velocity + self.right_velocity) / 2
            delta_distance_x = distance
            delta_distance_y = 0
            
        elif (orientation == 90):
            vel_msg.linear.y = abs((self.left_velocity + self.right_velocity) / 2)
            delta_distance_x = 0
            delta_distance_y = distance

        elif (orientation == 180):
            vel_msg.linear.x = -((self.left_velocity + self.right_velocity) / 2)
            delta_distance_x = -(distance)
            delta_distance_y = 0 

        elif (orientation == 270):
            vel_msg.linear.y = -((self.left_velocity + self.right_velocity) / 2)
            delta_distance_x = 0
            delta_distance_y = -(distance)

        else:
            delta_distance_x = 0
            delta_distance_y = 0
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

        # current_distance = 0
        right_dist=0
        left_dist=0
        
        right = self.right_position
        left = self.left_position
        # difference = 0
        condition_right = (right + distance)
        condition_left = (left + distance)
        
        while(right_dist <= condition_right and left_dist <= condition_left):
            if (isFree==True and isBumped == False):
                self.service_set_motor_velocity_left.call(self.left_velocity)
                self.service_set_motor_velocity_right.call(self.right_velocity)
                
                self.velocity_publisher.publish(vel_msg)
                
                self.service_set_motor_position_left.call(left + distance)
                self.service_set_motor_position_right.call(right + distance)
            
                right_dist = (self.right_position)
                left_dist = (self.left_position)
            
            else:
                self.service_set_motor_velocity_left.call(0)
                self.service_set_motor_velocity_right.call(0)
                self.velocity_publisher.publish(vel_msg)

        #After the loop, stops the robot
        x0 += delta_distance_x
        y0 += delta_distance_y
        x=x0
        y=y0
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0 #dovremmo mandare z angolare a 0 
        self.service_set_motor_velocity_left.call(0)
        self.service_set_motor_velocity_right.call(0)        
        self.velocity_publisher.publish(vel_msg)
        RobotPose[0]=x0
        RobotPose[1]=y0
        RobotPose[2]=yaw
        

    def bumper_callback(self, res):
        global isBumped, isFree

        if (res.data == True):
            isFree = False
            isBumped = True 
            
        else:
            isBumped=False
        

    def serve(self):
        self.service_set_torso_position(0.2)
        rospy.sleep(5)
        self.service_set_torso_position(0)
        rospy.sleep(5)

    def rotate(self, angle, clockwise):
        global x, y, yaw
        yaw0 = yaw
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 1

        # distanza che devono percorrere le ruote per compiere 90 gradi = 3.48 calcolato sperimentalmente
        distance90 = 3.525 
        distance = 0

        if angle == 45:
            distance = distance90/2
        elif angle == 90:
            distance = distance90
        elif angle == 135:
            distance = distance90 + distance90/2
        elif angle == 180:
            distance = distance90*2 +0.02*2
        elif angle == 225:
            distance = distance90*2 + distance90/2 +0.025*2
        elif angle == 270:
            distance = distance90*3 + 0.03*3
        elif angle == 315:
            distance = distance90*3 + distance90/2 + 0.035*3
        elif angle == 0:
            distance = (distance90 + 0.04)*4
        else: 
            rospy.logerr(f'angolo non accettato: {angle}' )

        right_dist = self.right_position
        left_dist = self.left_position
        
        right = self.right_position
        left = self.left_position
        
        if (clockwise==True):
            left_condition = left + distance
            right_condition = right - distance

            while (left_dist < left_condition and right_dist > right_condition):

                    self.service_set_motor_velocity_left.call(1)
                    self.service_set_motor_velocity_right.call(1)

                    self.service_set_motor_position_left.call(left_condition)
                    self.service_set_motor_position_right.call(right_condition)
                    
                    left_dist = self.left_position
                    right_dist = self.right_position

                    #Publish the velocity
                    self.velocity_publisher.publish(vel_msg)

            yaw0 -= angle
            yaw = (yaw0 % 360)
            
        
        else: 
            left_condition = left - distance
            right_condition = distance + right

            while (left_dist > left_condition and right_dist < right_condition):

                    self.service_set_motor_velocity_left.call(1)
                    self.service_set_motor_velocity_right.call(1)

                    self.service_set_motor_position_left.call(left_condition)
                    self.service_set_motor_position_right.call(right_condition)
                    
                    left_dist = self.left_position
                    right_dist = self.right_position

                    #Publish the velocity
                    self.velocity_publisher.publish(vel_msg)

            yaw0 += angle
            yaw = (yaw0 % 360)

        #After the loop, stops the robo
        vel_msg.angular.z = 0
        self.service_set_motor_velocity_left.call(0)
        self.service_set_motor_velocity_right.call(0)
        self.service_set_motor_position_left.call(0)
        self.service_set_motor_position_right.call(0)
        RobotPose[2]=yaw
        
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

        rospy.wait_for_service(self.name + "/torso_lift_joint_sensor/enable")
        rospy.wait_for_service(self.name + "/torso_lift_joint/set_position")
        rospy.wait_for_service(self.name + "/keyboard/enable")
        

    def create_services(self):
        self.service_set_motor_velocity_left = rospy.ServiceProxy(self.name + "/wheel_left_joint/set_velocity", set_float)
        self.service_set_motor_velocity_right = rospy.ServiceProxy(self.name + "/wheel_right_joint/set_velocity", set_float)

        self.service_enable_motor_position_left = rospy.ServiceProxy(self.name + "/wheel_left_joint_sensor/enable", set_int)
        self.service_enable_motor_position_right = rospy.ServiceProxy(self.name + "/wheel_right_joint_sensor/enable", set_int)

        self.service_set_motor_position_left = rospy.ServiceProxy(self.name + "/wheel_left_joint/set_position", set_float)
        self.service_set_motor_position_right = rospy.ServiceProxy(self.name + "/wheel_right_joint/set_position", set_float)
        self.service_get_target_position_left = rospy.ServiceProxy(self.name + "/wheel_left_joint/get_target_position", get_float)
        self.service_get_target_position_right = rospy.ServiceProxy(self.name + "/wheel_right_joint/get_target_position", get_float)

        self.service_accelerometer = rospy.ServiceProxy(self.name + "/accelerometer/enable", set_int)
        self.service_gyro = rospy.ServiceProxy(self.name + "/gyro/enable", set_int)
        self.service_inertial = rospy.ServiceProxy(self.name + "/inertial_unit/enable", set_int)
        self.service_lidar = rospy.ServiceProxy(self.name + "/Hokuyo_URG_04LX_UG01/enable", set_int)
        self.service_touch_sensor = rospy.ServiceProxy(self.name + "/base_cover_link/enable", set_int)
        self.torso_lift_joint_sensor = rospy.ServiceProxy(self.name + "/torso_lift_joint_sensor/enable", set_int)
        self.service_set_torso_position = rospy.ServiceProxy(self.name + "/torso_lift_joint/set_position", set_float)
        self.keyboard = rospy.ServiceProxy(self.name + "/keyboard/enable", set_int)

    def call_services(self):
        self.service_accelerometer.call(10)
        self.service_gyro.call(10)
        self.service_inertial.call(10)
        self.service_lidar.call(10)
        self.service_touch_sensor.call(10)
        self.torso_lift_joint_sensor.call(10)
        self.keyboard.call(10)
        self.service_enable_motor_position_left.call(10)
        self.service_enable_motor_position_right.call(10)
        


if __name__ == '__main__':
    x = 0 
    y = 0
    yaw = 0 
    isFree=True
    isBumped = False
    RobotPose=[x,y,yaw]
    
    try:
        TiagoController("/foo")
    except rospy.ROSInterruptException: 
        pass