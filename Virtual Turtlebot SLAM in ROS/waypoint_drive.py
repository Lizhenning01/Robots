#!/usr/bin/env python

import math
import roslib
import rospy
import tf
from rospy import ServiceProxy, ServiceException
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Twist, Quaternion
from ros_utilities import *
from nav_msgs.srv import GetPlan
import time
import path_planner as pp
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModelRequest, DeleteModel


class WaypointDrive:

    def __init__(self):

        # Nav values
        self.rate = 60
        self.d_gain = 1.1
        self.h_gain = 2.0
        self.v_max = 0.05
        self.w_max = 0.8
        self.margin = 0.06

        # odometry
        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0
        # for easy conversion
        self.orientation = [0, 0, 0, 0]
        self.position = [0, 0, 0]

        rospy.init_node("waypoint_drive", anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to_goal)

        self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ros_rate = rospy.Rate(self.rate)
        rospy.loginfo("Waypoint server node ready.")

    def odom_update(self, odom_msg):
        self.odom_x, self.odom_y, self.odom_yaw = pose_to_xyh(odom_msg.pose.pose)
        self.orientation = odom_msg.pose.pose.orientation
        self.position = odom_msg.pose.pose.position

    def go_to_goal(self, goal_msg):

        # rospy.wait_for_service('plan_path')
        startStamped = PoseStamped()
        startPose = Pose()
        startStamped.header.frame_id = "odom"
        startPose.position = self.position
        startPose.orientation = self.orientation
        startStamped.pose = startPose
        startStamped.header.stamp = rospy.Time.now()

        setup = GetPlan()
        setup.start = startStamped
        setup.goal = goal_msg
        setup.tolerance = .5

        rospy.loginfo("Executing go_to_goal to " + str([goal_msg.pose.position.x, goal_msg.pose.position.y]))

        request_waypoints = rospy.ServiceProxy('plan_path', GetPlan)

        try:
            received_wp_msg = request_waypoints(setup.start, setup.goal, setup.tolerance)
        except rospy.ServiceException, e:
            rospy.logerr("Path_plan cancelling: %s" % e)
            return

        waypoints = received_wp_msg.plan.poses


        if waypoints is None:
            rospy.logerr("Destination point unreachable.")
            return

        # driving time
        for waypoint in waypoints:
            # parse out pose from posestamped
            waypoint = waypoint.pose
            # get values we actually care about
            waypoint_x, waypoint_y, waypoint_yaw = pose_to_xyh(waypoint)

            # waypoint_x = -waypoint_x
            # waypoint_y = -waypoint_y
            rospy.loginfo("Going to: " + str(waypoint_x) + ", " + str(waypoint_y) + ", facing " + str(waypoint_yaw))
            # orient towards goal
            # calculate desired direction from x and y
            move_dir = math.atan2((waypoint_y - self.odom_y), (waypoint_x - self.odom_x))
            # rospy.loginfo("Calculated initial heading: " + str(move_dir))
            self.smooth_rotate(move_dir, 2)
            # rospy.sleep(.1)

            # move specific distance towards goal
            dist = math.sqrt((waypoint_x - self.odom_x) ** 2 + (waypoint_y - self.odom_y) ** 2)
            speed = .5

            self.smooth_drive(dist, speed)

            # face direction declared in pose msg
            # self.smooth_rotate(waypoint_yaw, 3)
        rospy.loginfo("Pathfinding concluded. Current location: " + str([self.odom_x, self.odom_y]))

    def smooth_rotate(self, angle_set, rotation_speed):
        """
        :param angle_set     [float] [rad]   The angle the robot should face.
        :param rotation_speed [float] [rad/s] The maximum rotational speed.
        """
        # rospy.loginfo("------- Smooth drive starting --------")

        error = angle_set - self.odom_yaw
        tolerance = 0.11

        start_time = time.time()
        accelerate_time = 1.0
        coeff = 1.0

        while abs(error) >= tolerance:
            runtime = time.time() - start_time
            if runtime < accelerate_time:
                coeff = math.sin(math.pi / 2 - (math.pi / 2 * ((accelerate_time - runtime) / accelerate_time)))
            if abs(error) < 1.5:
                rotation_speed = 2.0
                coeff = math.sin((math.pi / 2 * abs(error)))

            scaled_speed = rotation_speed * coeff

            if abs(scaled_speed) < 0.1:
                scaled_speed = 0.05

            if error < 0:
                scaled_speed = -scaled_speed
            self.send_speed(0, scaled_speed)
            rospy.sleep(.01)
            error = angle_set - self.odom_yaw

            # rospy.loginfo("Speed: " + str(scaled_speed))
            # rospy.loginfo("Current error: " + str(error))

        self.send_speed(0, 0)
        stopped_at = self.odom_yaw
        # rospy.sleep(1)
        # rospy.loginfo("------- Smooth rotate finished --------")
        # rospy.loginfo("Stopped at angle: " + str(stopped_at))
        # rospy.loginfo("Additional rotation after stopping: " + str(abs(self.odom_yaw) - abs(stopped_at)))
        rospy.loginfo("Rotation error: " + str(abs(self.odom_yaw) - abs(angle_set)))

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # rospy.loginfo("------- Smooth drive starting --------")

        initial_pose = [self.odom_x, self.odom_y]

        traveled = math.sqrt((initial_pose[0] - self.odom_x) ** 2 + (initial_pose[1] - self.odom_y) ** 2)
        tolerance = linear_speed / 50.0

        start_time = time.time()
        accelerate_time = .5
        coeff = 1
        self.send_speed(0, 0)

        while traveled <= distance - tolerance:
            runtime = time.time() - start_time
            if runtime < accelerate_time:
                coeff = math.sin(math.pi / 2 - (math.pi / 2 * ((accelerate_time - runtime) / accelerate_time)))
            elif distance - traveled < linear_speed + 3:
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

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        # Create twist message
        msg_cmd_vel = Twist()
        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        ### Publish the message
        self.move_pub.publish(msg_cmd_vel)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


def spawn():
    req = SpawnModelRequest()
    req.model_name = "model"
    req.model_xml = "/home/rbe3002/catkin_ws/src/turtlebot3/turtlebot3_description/rviz/model.rviz"
    req.robot_namespace = "model"
    req.initial_pose.position.x = 0.0
    req.initial_pose.position.y = 0.0
    req.initial_pose.position.z = 0.0
    req.initial_pose.orientation.x = 0.0
    req.initial_pose.orientation.y = 0.0
    req.initial_pose.orientation.z = 0.0
    req.initial_pose.orientation.w = 1.0
    req.reference_frame = ''

    try:
        srv = ServiceProxy('/gazebo/set_model_state', SpawnModel)
        resp = srv(req)
    except ServiceException, e:
        print("   Service call failed: %s" % e)
        return

    if resp.success:
        print(resp.status_message)
        return 0
    else:
        print(resp.status_message)
        return 1


def delete():
    try:
        srv = ServiceProxy('/gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        req.model_name = "turtlebot3_burger"
        resp = srv(req)
    except ServiceException, e:
        print("Service call failed: %s" % e)
        return

    if resp.success:
        print(resp.status_message)
        return 0
    else:
        print(resp.status_message)
        return 1


if __name__ == '__main__':
    # delete()
    # rospy.sleep(.1)
    # spawn()
    WaypointDrive().run()
