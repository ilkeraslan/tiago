#!/usr/bin/env python

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
"""

import rospy
from std_msgs.msg import Float64
from controller import Robot
import os


def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


robot = Robot()
timeStep = int(robot.getBasicTimeStep())
left = robot.getMotor('motor.left')
right = robot.getMotor('motor.right')
sensor = robot.getDistanceSensor('prox.horizontal.2')  # front central proximity sensor
sensor.enable(timeStep)
left.setPosition(float('inf'))  # turn on velocity control for both motors
right.setPosition(float('inf'))

velocity = 0
left.setVelocity(velocity)
right.setVelocity(velocity)
message = ''
print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
robot.step(timeStep)

rospy.init_node('listener', anonymous=True)
print('Subscribing to "motor" topic')
robot.step(timeStep)
rospy.Subscriber('motor', Float64, callback)
pub = rospy.Publisher('sensor', Float64, queue_size=10)
print('Running the control loop')
while robot.step(timeStep) != -1 and not rospy.is_shutdown():
    pub.publish(sensor.getValue())
    print('Published sensor value: ', sensor.getValue())
    if message:
        print(message)
        message = ''
    left.setVelocity(velocity)
    right.setVelocity(velocity)
