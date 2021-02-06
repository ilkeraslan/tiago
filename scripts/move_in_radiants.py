def move(self, distance):
        vel_msg = Twist()
        vel_msg.linear.x = (self.left_velocity + self.right_velocity) / 2
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = (self.left_velocity + self.right_velocity) / 2

        current_distance = 0
        delta_distance = (self.left_position + self.right_position) / 2
        t0 = rospy.Time.now().to_sec()

        while(current_distance < abs(distance - delta_distance)):
            self.service_set_motor_velocity_left.call(self.left_velocity)
            self.service_set_motor_velocity_right.call(self.right_velocity)

            self.service_set_motor_position_left.call(distance)
            self.service_set_motor_position_right.call(distance)
            
            self.velocity_publisher.publish(vel_msg)
            
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()

            current_distance = ((self.left_velocity + self.right_velocity) / 2) * (t1-t0)

        #After the loop, stops the robot
        vel_msg.linear.x = 0
        self.service_set_motor_velocity_left.call(self.left_velocity)
        self.service_set_motor_velocity_right.call(self.right_velocity)
        
        #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)

        # #Setting the current time for distance calculus
        # t0 = rospy.Time.now().to_sec()
        # current_distance = self.left_position
        # # new_distance = self.left_position + distance
        # # rot = distance / (math.pi * 0.2)

        # # pos_.rx() += std::cos(orient_) * req.linear;
        # # pos_.ry() += - std::sin(orient_) * req.linear;
        
        # # rotation = distance / (math.pi * 0.2)

        # new_distance = (self.left_position / 10) + distance
        # rotation = new_distance / (math.pi * 0.2)
        # angle = rotation * 2 * math.pi
        # left = self.left_velocity + angle
        # right = self.right_velocity + angle

        # # new_distance = (self.left_position / 10) + distance
        # # angle = new_distance * 10
        # # left = self.left_velocity + angle
        # # right = self.right_velocity + angle

        # while(current_distance < ((distance + 0.1) * 10)):
        #     self.service_set_motor_velocity_left.call(self.left_velocity)
        #     self.service_set_motor_velocity_right.call(self.right_velocity)

        #     self.service_set_motor_position_left.call(left)
        #     self.service_set_motor_position_right.call(right)
            
        #     #Publish the velocity
        #     self.velocity_publisher.publish(vel_msg)
            
        #     #Takes actual time to velocity calculus
        #     t1=rospy.Time.now().to_sec()

        #     current_distance = ((self.left_velocity + self.right_velocity) / 2) * (t1-t0)
        #     first_distance = self.left_position + current_distance
        #     rospy.logerr(f'CURRENT DISTANCE: {current_distance}')
        
        # #After the loop, stops the robot
        # vel_msg.linear.x = 0
        # self.service_set_motor_velocity_left.call(self.left_velocity)
        # self.service_set_motor_velocity_right.call(self.right_velocity)
        
        # #Force the robot to stop
        # self.velocity_publisher.publish(vel_msg)