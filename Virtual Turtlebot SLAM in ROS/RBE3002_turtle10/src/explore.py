#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Twist
from laser_geometry import LaserProjection
import math
from math import sin, cos, pi
import random
import traceback
import time


class Explore:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('explore', anonymous=True)
        self.laser = LaserScan()
        self.laser_proj = LaserProjection()
        self.pc_pub = rospy.Publisher('/laserPointCloud', PointCloud2, queue_size=1)
        self.sacn_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.front_thresh = 0.3
        

    def callback(self, msg):
        'inside callback'
        ranges = list(msg.ranges)
        cloud_out = self.laser_proj.projectLaser(msg)

        # front_thresh = 0.27
        side_thresh = 0.5
        tolerance = 0.38

        right_dist_average = 0
        left_dist_average = 0

        front_dist = 100

        if len(ranges) != 0:
            right_dist = ranges[45:134]
            left_dist = ranges[225:314]

            if ranges[0] < 100000:
                front_dist_list = ranges[349:359]
                front_dist_list.extend(ranges[0:10])

                front_dist = sum(front_dist_list)/len(front_dist_list)

            right_dist_average = sum(right_dist)/len(right_dist)
            left_dist_average = sum(left_dist)/len(left_dist)

        diff = abs(right_dist_average - left_dist_average)
        
        right = False
        left = False

        if right_dist_average > left_dist_average and diff > tolerance:
            left = True

        if left_dist_average > right_dist_average and diff > tolerance:
            right = True

        if (right_dist_average > side_thresh) and (left_dist_average > side_thresh):
            left = True
            right = True

        #-------------------------------------------------------------------------------------------

        if front_dist>self.front_thresh:
            print 'drive straight', front_dist, self.front_thresh
            if front_dist > 1:
                self.front_thresh = 0.27
            self.send_speed(1,0)

        else:
            if left and (not right):
                self.front_thresh = front_dist-0.05
                print 'turn left', front_dist, self.front_thresh
                self.rotate(pi/2, 1)
           
            if right and (not left):
                self.front_thresh = front_dist-0.05
                print 'turn right', front_dist, self.front_thresh
                self.rotate(pi/2, -1)
               
            if (not right) and (not left):
                self.front_thresh = front_dist-0.05
                print 'drive back', front_dist, self.front_thresh
                self.rotate(pi, 1)

            if left and right:
                x = random.randint(0,1)
                if x == 0:
                    print 'turn left rand', front_dist, self.front_thresh
                    self.front_thresh = front_dist-0.05
                    self.rotate(pi/2, 1)
                else:
                    print 'turn right rand', right_dist_average, left_dist_average, diff
                    self.front_thresh = front_dist-0.05
                    self.rotate(pi/2, -1)

                
    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # REQUIRED CREDIT
        # Make a new Twist message
        # TODO                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        self.msg_cmd_vel = Twist()
        #Linear velocity
        self.msg_cmd_vel.linear.x=linear_speed
        self.msg_cmd_vel.linear.y=0.0
        self.msg_cmd_vel.linear.z=0.0
        #Angular Velocity
        self.msg_cmd_vel.angular.x = 0.0
        self.msg_cmd_vel.angular.y = 0.0

        if angular_speed < 0:
            clockwise = True
        else:
            clockwise = False

        if clockwise:
            self.msg_cmd_vel.angular.z = -abs(angular_speed)
        else:
            self.msg_cmd_vel.angular.z = abs(angular_speed)


        # Publish the message
        # TODO
        self.velocity_publisher.publish(self.msg_cmd_vel)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """

        # REQUIRED CREDIT
        init_time = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < abs(angle):
            self.send_speed(0,aspeed)
            time = rospy.Time.now().to_sec()
            if aspeed > 0:
                current_angle = aspeed*(time - init_time)
            else:
                current_angle = -aspeed*(time - init_time)
        self.msg_cmd_vel.angular.z = 0
        self.velocity_publisher.publish(self.msg_cmd_vel)
        # new_laser = LaserScan()
        # self.callback(new_laser)

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # rospy.loginfo("------- Smooth drive starting --------")

        initial_pose = [self.odom_x, self.odom_y]

        traveled = math.sqrt((initial_pose[0] - self.odom_x) ** 2 + (initial_pose[1] - self.odom_y) ** 2)
        tolerance = linear_speed / 50.0

        coeff = 1
        self.send_speed(0, 0)
        while traveled <= distance - tolerance:
            # accelerate linearly
            while traveled < 0.2:
                self.send_speed(traveled + .2, 0)

            # decelerate with sin() curve
            if distance - traveled < linear_speed + 3:
                linear_speed = 1
                coeff = math.sin(math.pi / 2 + (math.pi / 2 * (traveled / distance)))

            scaled_speed = linear_speed * coeff
            if scaled_speed < 0.1:
                scaled_speed = 0.1
            self.send_speed(scaled_speed, 0)
            rospy.sleep(.05)
            traveled = math.sqrt((initial_pose[0] - self.odom_x) ** 2 + (initial_pose[1] - self.odom_y) ** 2)

            # rospy.loginfo("Scaled speed: " + str(scaled_speed))
            # rospy.loginfo("Total distance traveled: " + str(traveled))

        self.send_speed(0, 0)
        # rospy.sleep(1)
        stopped_at = traveled
        traveled = math.sqrt((initial_pose[0] - self.odom_x) ** 2 + (initial_pose[1] - self.odom_y) ** 2)
        # rospy.loginfo("------- Smooth drive finished --------")
        # rospy.loginfo("Desired distance: " + str(distance))
        # rospy.loginfo("Total distance after stopping: " + str(traveled))
        # rospy.loginfo("Additional distance after stopping: " + str(traveled - stopped_at))
        rospy.loginfo("Distance error: " + str(traveled - distance))



    def run(self):
        self.callback(LaserScan())

        rospy.spin()

if __name__ == "__main__":
    
    explore = Explore()
    explore.run()