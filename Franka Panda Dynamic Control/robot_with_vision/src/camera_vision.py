#!/usr/bin/env python

#Taylor Bergeron

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import numpy
from geometry_msgs.msg import Pose
import imutils
import numpy as np
import cv2
from robot_with_vision.msg import centers, points_to_go_to
from cv_bridge import CvBridge, CvBridgeError


class camera_vision:

    def __init__(self):
        #self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        #self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_image_callback)
        self.depth_points_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.depth_points_callback)
        self.box_center_sub = rospy.Subscriber('/box_centers', centers, self.box_center_callback_pts)

        self.position_publisher = rospy.Publisher("/block_points", points_to_go_to, queue_size=10)

        self.color_list = []
        self.depth_points_array = []
        self.x_centers = []
        self.y_centers = []
        self.z_centers = [0,0,0] # DEPTH

        self.x_positions = list()
        self.y_positions = list()
        self.z_positions = list()

        #self.bridge = CvBridge()


    def depth_image_callback(self, data):
        self.depth_points_array = data.data
        #rospy.logwarn(data.encoding)

    def depth_points_callback(self, data):
        # height: 480 width: 640
        self.depth_full = data
        self.depth_points_array = data.data
        #rospy.logwarn(data.fields)

    def box_center_callback_img(self, data):
        self.x_centers = data.x_centers
        self.y_centers = data.y_centers
        self.color_list = data.colors
        #
        # if self.depth_points_array is not None:
        #     for i in range(len(self.x_centers)):
        #         x_pt = int(self.x_centers[i])
        #         y_pt = int(self.y_centers[i])
        #
        #         index = (x_pt*480 + y_pt)*4
        #
        #         rospy.logwarn(self.depth_points_array[index])
                #z = self.depth_points_array[y_pt][x_pt]

    def box_center_callback_pts(self, data):
        self.x_centers = data.x_centers
        self.y_centers = data.y_centers
        #self.z_centers = []
        #rospy.logwarn(self.x_centers)

        self.x_positions = []
        self.y_positions = []
        self.z_positions = []

        for i in range(len(self.x_centers)):

            if self.depth_full is not None:
                self.gen = pc2.read_points(self.depth_full, field_names=("x", "y", "z"), skip_nans=False)

            x_pt = int(self.x_centers[i])
            y_pt = int(self.y_centers[i])

            index = y_pt*640 + x_pt
            #rospy.logwarn(index)

            counter = 0
            if self.gen is not None:
                for p in self.gen:
                    if counter == index:
                        # rospy.logwarn(index)
                        # rospy.logwarn(p[0])
                        # rospy.logwarn(p[1])
                        # rospy.logwarn(p[2])
                        self.x_positions.append(str(p[0]+0.001731))
                        self.y_positions.append(str(p[1]+0.036))

                        # MIGHT NEED TO ADD 0.05m to the Z!!!!
                        self.z_positions.append(str(p[2]+0.237283))

                    counter = counter + 1
                    # rospy.logwarn(len(self.gen))
                    # rospy.logwarn(p)
                #rospy.logwarn(counter)
                # a_pt = self.gen[x_pt][y_pt]
                # rospy.logwarn(a_pt)
        # rospy.logwarn(self.x_positions)
        # rospy.logwarn(self.y_positions)
        # rospy.logwarn(self.z_positions)
        # rospy.logwarn(data.colors)

        points = points_to_go_to()
        points.x_points = self.z_positions
        points.y_points = self.x_positions
        points.z_points = self.y_positions
        points.colors = data.colors

        # In robots base frame the camera is at: 0.23728,0.001731,0.036

        self.position_publisher.publish(points)


def main(args):
  cv = camera_vision()
  rospy.init_node('get_box_coordinates', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
