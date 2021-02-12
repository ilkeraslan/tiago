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
from webots_ros.msg import Float64Stamped, BoolStamped
from geometry_msgs.msg import Point, Twist, Vector3
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
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
        self.left_position_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_left_joint_sensor/value', Float64Stamped, self.position_callback_left) #distanza percorsa dalla ruota sx
        self.right_position_sensor_subscriber = rospy.Subscriber(self.name + '/wheel_right_joint_sensor/value', Float64Stamped, self.position_callback_right)

        self.left_velocity = 1
        self.right_velocity = 1
        self.left_position = 0
        self.right_position = 0
        self.orientation = 0 #0, 90, 180, 270

        rospy.logerr(f'Left post: {self.left_position}')
        rospy.logerr(f'Right post: {self.right_position}')
        

        
        rospy.sleep(4)
        self.move(20,0)
        rospy.sleep(2)
        self.rotate(180)
        rospy.sleep(2)
        self.move(20, 180)
        rospy.sleep(2)
        
        # self.rotate(45)
        # rospy.sleep(2)
        # self.rotate(180)
        # rospy.sleep(2)
        # self.rotate(90)
        # rospy.sleep(2)
        # self.rotate(225)
        # rospy.sleep(2)
        # self.rotate(270)
        # rospy.sleep(2) 
        # self.rotate(125)
        # rospy.sleep(2) 
        # self.rotate(315)

        # self.rotate()
        # rospy.sleep(2)
        # rospy.logerr(f'Left post: {self.left_position}')
        # rospy.logerr(f'Right post: {self.right_position}')
        # self.move(10, 0)
        # rospy.sleep(2)
        # self.rotate(180)
        # rospy.sleep(2)
        # self.move(20, 180)
        # rospy.sleep(2)
        # self.rotate(90)
        # rospy.sleep(2)
        # self.move(14, 270)
        # rospy.sleep(2)
        # self.rotate(90)
        # rospy.sleep(2)
        # self.move(14, 0)
        # rospy.sleep(2)
        # self.rotate(90)
        # rospy.sleep(2)
        # self.move(14, 90)
        # rospy.sleep(2)
        # self.rotate(90)
        # rospy.sleep(2)
        # self.move(4, 180)
        # rospy.sleep(2)
        # self.rotate(180)
        # rospy.sleep(2)
        # self.rotate(0)


        rospy.logerr(f'Left post: {self.left_position}')
        rospy.logerr(f'Right post: {self.right_position}')
        rospy.sleep(2)
    


        rospy.spin()


    def bumper_callback(self, res):
        pass
        # rospy.logwarn(f'Bumped: {res.data}')
        # if (res.data is True):
        #     self.service_set_motor_velocity_left.call(1)
        #     self.service_set_motor_velocity_right.call(1)

    def move(self, distance, orientation):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = (self.left_velocity + self.right_velocity) / 2 #sbagliata

        if (orientation == 0):
            vel_msg.linear.x = (self.left_velocity + self.right_velocity) / 2
        elif (orientation == 90):
            vel_msg.linear.y = (self.left_velocity + self.right_velocity) / 2
        elif (orientation == 180):
            vel_msg.linear.x = -((self.left_velocity + self.right_velocity) / 2)
        elif (orientation == 270):
            vel_msg.linear.y = -((self.left_velocity + self.right_velocity) / 2)
        else:
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
        
        while(right_dist <= condition_right and left_dist <= condition_left ):
            
            self.service_set_motor_velocity_left.call(self.left_velocity)
            self.service_set_motor_velocity_right.call(self.right_velocity)
            self.service_set_motor_position_left.call(left + distance)
            self.service_set_motor_position_right.call(right + distance)
            self.velocity_publisher.publish(vel_msg)
            # time_start += rospy.Time.now().to_sec()
            right_dist = (self.right_position)
            left_dist = (self.left_position)

        #After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0 #dovremmo mandare z angolare a 0 
        self.service_set_motor_velocity_left.call(0)
        self.service_set_motor_velocity_right.call(0)        
        self.velocity_publisher.publish(vel_msg)

    
    def rotate(self, angle):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

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

        # rospy.logwarn(f'LEFT POSITION: {self.left_position}')
        # rospy.logwarn(f'RIGHT POSITION: {self.right_position}')
        rospy.logwarn(f'LEFT: {self.left_position}')
        rospy.logwarn(f'left_dist: {left_dist}')
        # rospy.logwarn(f'RIGHT WHEEL: {right_wheel}')

        # rotation = distance / (math.pi * 0.2)
        # angle = rotation * 2 * math.pi
        # left = 0
        # right = speed + angle + 6
        # rospy.logwarn(f'RIGHT: {right}')

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
        
        #After the loop, stops the robot
        vel_msg.angular.z = 0
        self.service_set_motor_velocity_left.call(0)
        self.service_set_motor_velocity_right.call(0)
        self.service_set_motor_position_left.call(0)
        self.service_set_motor_position_right.call(0)
        
        #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)



    # def rotate(self, angle, speed=100):

    #     vel_msg = Twist()

    #     clockwise = True if angle < 0 else False

    #     # Converting from angles to radians
    #     relative_angle = np.deg2rad(abs(angle))
    #     angular_speed = np.deg2rad(speed)

    #     if clockwise:
    #         angular_speed = -angular_speed

    #     # Setting the current time for distance calculus
    #     t0 = rospy.Time.now().to_sec()
    #     current_angle = 0

    #     while not rospy.is_shutdown() and current_angle < relative_angle:
    #         self.move(0, angular_speed)
    #         t1 = rospy.Time.now().to_sec()
            # current_angle = abs(angular_speed)*(t1-t0)

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
