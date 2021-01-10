#!/usr/bin/env python

import rospy
import math
from utils import ThymioController



class ThymioController_Task3(ThymioController):

    def run(self):
        """Controls the Thymio."""

        ### TASK 2

        wall_distance = 0.08 # meters

        rospy.loginfo('Moving forward...')

        while not rospy.is_shutdown() and not self.collision(wall_distance):
            self.move()
            self.rate.sleep()

        self.stop()
        rospy.loginfo('CHECKPOINT Task 2: Collision (%.2f meters distant from the wall)' % wall_distance)
        rospy.loginfo('Trying to face the wall exactly...')

        while not rospy.is_shutdown() and not self.is_exactly_facing_an_obstacle():
            if self.proximity_left.range <= self.proximity_right.range:
                self.move(0, 0.2) # turning left
            else:
                self.move(0, -0.2) # turning right

        self.stop()
        rospy.loginfo('CHECKPOINT Task 2: Facing the wall exactly')
        rospy.loginfo('Waiting a few seconds...')
        rospy.sleep(2) # sleep for 2 seconds

        ### TASK 3

        rospy.loginfo('Turning by 180 degrees...')
        self.rotate(180)

        self.stop()
        rospy.loginfo('CHECKPOINT Task 3: The wall is exactly behind')

        from_wall = 2
        (init_x, init_y, init_theta) = self.human_readable_pose2d(self.pose)
        goal_x = from_wall * math.cos(init_theta) + init_x
        goal_y = from_wall * math.sin(init_theta) + init_y

        rospy.loginfo('Start walking for having the wall %.2f meters far behind...' % from_wall)
        rospy.loginfo('Starting pose is: \t init_x %f \t init_y %f' % (init_x, init_y))

        while not rospy.is_shutdown() and self.euclidean_distance_2d(goal_x, goal_y) >= wall_distance:
            self.move()

        self.stop()

        (final_x, final_y, final_theta) = self.human_readable_pose2d(self.pose)
        rospy.loginfo('Final pose is: \t\t final_x %f \t final_y %f' % (final_x, final_y))
        rospy.loginfo('%f meters distant from the starting pose (the one near the wall)' % self.euclidean_distance_2d(init_x, init_y))
        rospy.loginfo('CHECKPOINT Task 3: This means the robot is almost 2 meters distant from the wall')




if __name__ == '__main__':

    controller = ThymioController_Task3('assignment2_task3')

    try:

        controller.run()


    except rospy.ROSInterruptException as e:
        pass