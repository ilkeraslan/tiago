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

        if(front_distance < 0.4):
            rospy.logerr("new-OBSTACLE")
            isFree = False 
            rospy.sleep(5)
            
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
        

        self.move(19, 0)
        rospy.sleep(1)
        self.rotate(90)
        rospy.sleep(1)
        self.move(19, 90)
        rospy.sleep(1)
        self.rotate(270)
        rospy.sleep(1)
        self.move(20, 0)
        rospy.sleep(1)
        self.goalRaggiunto()

        rospy.spin()

    def keyboard_callback(self, res):
        key=res.data
        if (key == 65):
            self.move_to_a()
            rospy.sleep(1)

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

    def move(self, distance, orientation):
        global x,y,yaw, isFree
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
            if (isFree==True):
                self.service_set_motor_velocity_left.call(self.left_velocity)
                self.service_set_motor_velocity_right.call(self.right_velocity)
                
                self.velocity_publisher.publish(vel_msg)
                
                self.service_set_motor_position_left.call(left + distance)
                self.service_set_motor_position_right.call(right + distance)
            
                right_dist = (self.right_position)
                left_dist = (self.left_position)
            
            else:
                while(isFree==False):
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
        rospy.logerr(f"my pose: {x0} . {y0} - {yaw} ")

    def bumper_callback(self, res):
        pass
        # rospy.logwarn(f'Bumped: {res.data}')
        # if (res.data is True):
        #     self.service_set_motor_velocity_left.call(1)
        #     self.service_set_motor_velocity_right.call(1)

    def move1(self, distance, orientation):
        global x,y,yaw
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
            self.service_set_motor_velocity_left.call(self.left_velocity)
            self.service_set_motor_velocity_right.call(self.right_velocity)
            
            self.velocity_publisher.publish(vel_msg)
            
            self.service_set_motor_position_left.call(left + distance)
            self.service_set_motor_position_right.call(right + distance)
            
            # time_start += rospy.Time.now().to_sec()
            right_dist = (self.right_position)
            left_dist = (self.left_position)

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
        rospy.logerr(f"my pose: {x0} . {y0} - {yaw} ")

    def goalRaggiunto(self):
        self.service_set_torso_position(0.2)
        rospy.logerr("Sto alzando il torso")
        rospy.sleep(5)
        self.service_set_torso_position(0)
        rospy.sleep(5)
    
    def rotate(self, angle):
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
        distance90 = 3.48 
        distance = 0
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        if angle == 45:
            distance = distance90/2 -0.01
        elif angle == 90:
            distance = distance90
        elif angle == 135:
            distance = distance90 + distance90/2 + 0.01
        elif angle == 180:
            distance = distance90*2 +0.02*2 -0.01
        elif angle == 225:
            distance = distance90*2 + distance90/2 +0.025*2 - 0.01
        elif angle == 270:
            distance = distance90*3 +0.03*3 -0.01
        elif angle == 315:
            distance = distance90*3 + distance90/2 + 0.035*3 - 0.01
        elif angle == 0:
            distance = (distance90 + 0.04)*4 -0.02
        else: 
            rospy.logerr(f'angolo non accettato: {angle}' )

        right_dist = self.right_position
        left_dist = self.left_position
        
        right = self.right_position
        left = self.left_position
        rospy.logerr(f'left: {left}' )
        rospy.logerr(f'right: {right}' )
        
        left_condition = left - distance
        right_condition = distance + right
        rospy.logerr(f'left_condition: {left_condition}' )
        rospy.logerr(f'right_condition: {right_condition}' )
        # distance = math.sqrt(pow((x_start + 0.9 - x_start), 2) + pow((y_start - y_start), 2))
        speed = 5
        degrees = 90
        angular_speed = np.deg2rad(speed)
        relative_angle = np.deg2rad(abs(degrees))
        vel_msg.angular.z = angular_speed

        while (left_dist > left_condition and right_dist < right_condition):
            # rospy.logwarn(f'CURRENT: {current_angle}')
            # rospy.logwarn(f'RELATIVE: {relative_angle}')
            rospy.logwarn(f'LEFT POSITION: {self.left_position}')
            rospy.logwarn(f'RIGHT POSITION: {self.right_position}')

            # rospy.logwarn(f'LEFT WHEEL: {left_wheel}')
            # rospy.logwarn(f'RIGHT WHEEL: {right_wheel}')

            self.service_set_motor_velocity_left.call(1)
            self.service_set_motor_velocity_right.call(1)

            self.service_set_motor_position_left.call(left_condition)
            self.service_set_motor_position_right.call(right_condition)
            
            left_dist = self.left_position
            right_dist = self.right_position
            #Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            
            #Calculates distancePoseStamped
            current_angle = abs(angular_speed)*(t1-t0)
        
        #After the loop, stops the robo
        yaw0 += angle
        yaw = (yaw0 % 360)
        vel_msg.angular.z = 0
        self.service_set_motor_velocity_left.call(0)
        self.service_set_motor_velocity_right.call(0)
        self.service_set_motor_position_left.call(0)
        self.service_set_motor_position_right.call(0)
        
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
        # rospy.wait_for_service(self.name + "/wheel_left_joint/get_position")
        # rospy.wait_for_service(self.name + "/wheel_right_joint/get_position")

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
    
    try:
        TiagoController("/foo")
    except rospy.ROSInterruptException: 
        pass
