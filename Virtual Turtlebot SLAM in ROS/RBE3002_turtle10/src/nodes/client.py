#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import nav_msgs.srv
import nav_msgs
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from std_msgs.msg import String
import math

DRIVE_TOLERANCE = 0.08
ROTATE_TOLERANCE = 0.04


class Client:

    def __init__(self):

        # Initialize node, name it 'Client'
        rospy.init_node('Client', anonymous=True)

        # ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        # print('vel publisherr')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # ### When a message is received, call self.update_odometry
        rospy.Subscriber('odom', Odometry, self.update_odometry)

        # ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        # ### When a message is received, call self.go_to
        #rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.get_initial_pose)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)
        rospy.Subscriber('/move_base_simple/frontier_goal', PoseStamped, self.get_frontier_goal)

        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0
        self.init_pose = None
        self.goal = None
        rospy.loginfo("Client node ready.")

    def update_odometry(self, msg):
        #update odometry and update the start location
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_ori = msg.pose.pose.orientation
        quat_list = [quat_ori.x, quat_ori.y, quat_ori.z, quat_ori.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

        m = PoseStamped()
        m.header = msg.header
        m.pose = msg.pose.pose
        self.init_pose = m

    def get_goal(self, msg):
        # when receives a nav/goal, generate a path to that goal location
        self.goal = msg
        if (self.init_pose != None):
            path = self.plan_path()
            # clean up mem
            self.init_pose = None
            self.goal = None

            self.nav_to_goal(path)

    def get_frontier_goal(self, msg):
        # when receives a nav/frontier_goal, generate a path to that goal
        self.goal = msg
        if (self.init_pose != None):
            path = self.plan_path()
            # clean up mem
            self.init_pose = None
            self.goal = None

            self.nav_to_frontier_goal(path)

    def plan_path(self):
        rospy.loginfo("Nav goal set")
        # rospy.wait_for_service('a_star')
        try:
            aStar = rospy.ServiceProxy('a_star', GetPlan)
            path = aStar(self.init_pose, self.goal, 0.05)
            print path.plan.poses
            return path.plan.poses
        except rospy.ServiceException, e: 
            rospy.logerr(e)
            #rospy.logwarning("Service call failed: %s" %e)

    def nav_to_goal(self, path): 
        # Follow the poses in the path, go to the goal
        self.go_to(path[0])

        for c in range(len(path)):
            self.go_to(path[c])

    def nav_to_frontier_goal(self, path):
        # only go tho half of the path when exploring new map
        for c in range(len(path)/2):
            self.go_to(path[c])

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        msg_cmd_vel = Twist()
        #Linear velocity
        msg_cmd_vel.linear.x=linear_speed
        msg_cmd_vel.linear.y=0.0
        msg_cmd_vel.linear.z=0.0
        #Angular Velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        self.velocity_publisher.publish(msg_cmd_vel)

    def drive(self, distance, time):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param time [int] [s] The duration.
        """
        #initial pose
        init_x = self.px
        init_y = self.py
        init_yaw = self.pth
        #final destination
        final_x = distance * math.cos(init_yaw) + init_x
        final_y = distance * math.sin(init_yaw) + init_y
        #linear_speed = distance / float(time)
        if distance>=0:
            linear_speed = 0.08
        else:
            linear_speed = -0.08

        self.send_speed(linear_speed, 0.0)
        rospy.sleep(0.1)

        while self.distance(init_x, init_y, self.px, self.py) < (linear_speed / 10):
            self.send_speed(linear_speed, 0.0)
            rospy.sleep(0.1)        

        dist_togo = self.distance(self.px, self.py, final_x, final_y)
        min_dist = dist_togo
        while dist_togo > DRIVE_TOLERANCE and dist_togo <= 3*min_dist:
            if dist_togo < min_dist:
                min_dist = dist_togo
            
            print "driving, dist_togo: ", dist_togo
            dist_togo = self.distance(self.px, self.py, final_x, final_y)
            self.rate.sleep()
        self.send_speed(0.0, 0.0)

    def rotate(self, angle, time):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param time     [int] [s] The duration of the rotation.
        """

        init_yaw = self.pth
        final_yaw = init_yaw + angle

        aspeed = angle / float(time)
        if angle>=0:
            aspeed = 0.2
        else: 
            aspeed = -0.2

        self.send_speed(0.0, aspeed)
        rospy.sleep(0.1)

        while abs(self.pth - init_yaw) < (aspeed / 10):
            self.send_speed(0.0, aspeed)
            rospy.sleep(0.1)
        
        diff_angle = abs(angle)
        min_angle = diff_angle

        while diff_angle > ROTATE_TOLERANCE and abs(diff_angle - 2*math.pi) > ROTATE_TOLERANCE:
            diff_angle = abs(final_yaw - self.pth)
            self.rate.sleep()
        self.send_speed(0.0, 0.0)

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        angle = math.atan2((pos_y - self.py), (pos_x - self.px))

        if angle > math.pi:
            angle = -(2*math.pi - angle)
        elif angle < (-math.pi):
            angle = 2*math.pi + angle
        distance = math.sqrt( ((pos_x - self.px) ** 2) + ((pos_y - self.py) ** 2) )

        if distance > 0.08:
            self.smooth_rotate(angle, 1.5)
            # print "Finished rotating for", pos_x, pos_y
            self.smooth_drive(self.distance(self.px, self.py, pos_x, pos_y), .5)
            # print "Finished driving for", pos_x, pos_y

    def smooth_rotate(self, angle_set, rotation_speed):
        """
        :param angle_set     [float] [rad]   The angle the robot should face.
        :param rotation_speed [float] [rad/s] The maximum rotational speed.
        """
        # rospy.loginfo("------- Smooth rotate starting --------")

        initial_pose = self.pth
        error = angle_set - self.pth
        tolerance = 0.05

        coeff = 1.0
        traveled = 0
        while abs(error) >= tolerance:
            # accelerate linearly
            while abs(traveled) < 0.2:
                traveled = abs(self.pth - initial_pose)
                self.send_speed(0, 3 * traveled + .2)

            if abs(error) < 1.5:
                rotation_speed = 2.0
                coeff = math.sin((math.pi / 2 * abs(error)))

            scaled_speed = rotation_speed * coeff

            if abs(error) < 0.2:
                scaled_speed = 0.1

            self.send_speed(0, scaled_speed)
            rospy.sleep(.01)

            error = angle_set - self.pth

            # rospy.loginfo("Speed: " + str(scaled_speed))
            # rospy.loginfo("Current error: " + str(error))

        self.send_speed(0, 0)
        # stopped_at = self.pth
        # rospy.sleep(1)
        # rospy.loginfo("------- Smooth rotate finished --------")
        # rospy.loginfo("Stopped at angle: " + str(stopped_at))
        # rospy.loginfo("Additional rotation after stopping: " + str(abs(self.odom_yaw) - abs(stopped_at)))
        rospy.loginfo("Rotation error: " + str(abs(self.pth) - abs(angle_set)))

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # rospy.loginfo("------- Smooth drive starting --------")

        initial_pose = [self.px, self.py]

        traveled = math.sqrt((initial_pose[0] - self.px) ** 2 + (initial_pose[1] - self.py) ** 2)
        tolerance = linear_speed / 50.0

        coeff = 1
        self.send_speed(0, 0)
        while traveled <= distance - tolerance:
            # rospy.loginfo("Total distance traveled: " + str(traveled))

            # accelerate linearly
            while traveled < 0.2:
                traveled = math.sqrt((initial_pose[0] - self.px) ** 2 + (initial_pose[1] - self.py) ** 2)
                self.send_speed(traveled + .05, 0)
                rospy.sleep(.05)

            # decelerate with sin() curve
            if distance - traveled < linear_speed + 3:
                linear_speed = .5
                coeff = math.sin(math.pi / 2 + (math.pi / 2 * (traveled / distance)))

            scaled_speed = linear_speed * coeff
            if scaled_speed < 0.02:
                scaled_speed = 0.02
            self.send_speed(scaled_speed, 0)
            rospy.sleep(.05)
            traveled = math.sqrt((initial_pose[0] - self.px) ** 2 + (initial_pose[1] - self.py) ** 2)

            # rospy.loginfo("Scaled speed: " + str(scaled_speed))
            # rospy.loginfo("Total distance traveled: " + str(traveled))

        self.send_speed(0, 0)
        # rospy.sleep(1)
        stopped_at = traveled
        traveled = math.sqrt((initial_pose[0] - self.px) ** 2 + (initial_pose[1] - self.py) ** 2)
        # rospy.loginfo("------- Smooth drive finished --------")
        # rospy.loginfo("Desired distance: " + str(distance))
        # rospy.loginfo("Total distance after stopping: " + str(traveled))
        # rospy.loginfo("Additional distance after stopping: " + str(traveled - stopped_at))
        rospy.loginfo("Distance error: " + str(traveled - distance))

    @staticmethod
    def distance(x0, y0, x1, y1):
        return math.sqrt((x1-x0)**2 + (y1 -y0) **2)

    def run(self):
        #Client().nav_to_goal(PoseStamped())
        rospy.spin()


if __name__ == "__main__":
    c = Client()
    rospy.sleep(1)
    c.run()
