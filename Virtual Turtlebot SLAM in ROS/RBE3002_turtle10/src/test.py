#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped, PointStamped
import tf2_ros
import PyKDL
import tf2_geometry_msgs
from math import cos,sin
class LaserRot(object):
    def __init__(self):
        self.laser=LaserScan()
        self.laserS=rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.closestP=rospy.Publisher("/closest_point", PointStamped, queue_size=1)
        self.tf_buffer=tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer)


        self.get_transform()

    def get_transform(self):
        try:
            self.transform = self.tf_buffer.lookup_transform("base_link", "laser", rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerror("Error getting transform")
            print "Error"

    def laser_callback (self, msg):
        self.laser=msg
        self.get_transform()

    def publish_closest_obstacle(self):
        laser = self.laser.ranges
        shortest_laser = 10000
        point=Point()
        for i in range(len(laser)):
            if laser[i] < shortest_laser:
                angle=self.laser.angle_min + i*self.laser.angle_increment
                x=laser[i]*cos(angle)
                if x>-0.2:
                    shortest_laser=laser[i]
                    point.x=x
                    point.y=shortest_laser*sin(angle)
        pose=PoseStamped()
        pose.header=self.laser.header
        point.z=0.0
        pose.pose.position=point
        pose_transformed= tf2_geometry_msgs.do_transform_pose(pose, self.transform)
        point_transformed=PointStamped()


        point_transformed.header=pose_transformed.header
        point_transformed.point = pose_transformed.pose.position
        self.closestP.publish(point_transformed)

rospy.init_node("compute_closest_obstcl")
r=rospy.Rate(10)
lr=LaserRot()

while not rospy.is_shutdown():
    lr.publish_closest_obstacle()
    r.sleep()