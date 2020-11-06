#!/usr/bin/env python
# Taylor Bergeron

import sys
# sys.path.remove('/opt/ros/kinetic/lib/python3.6/dist-packages') # in order to import cv2 under python
import cv2
# sys.path.append('/opt/ros/kinetic/lib/python3.6/dist-packages') # append back in order to import rospy

import numpy as np
import roslib

roslib.load_manifest('robot_with_vision')
import rospy
from std_msgs.msg import String
from robot_with_vision.msg import centers
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils


# https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/

class image_converter:

    def __init__(self):
        self.centers_pub = rospy.Publisher("box_centers", centers, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):

        # height: 480 width: 640

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #   cv2.circle(cv_image, (50,50), 10, 255)
        #
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        hsvFrame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)

        blue_lower = np.array([94, 80, 2], np.uint8)
        blue_upper = np.array([120, 255, 255], np.uint8)

        # red_lower = np.array([136, 87, 111], np.uint8)
        # red_upper = np.array([180, 255, 255], np.uint8)

        red_lower = np.array([0, 50, 20])
        red_upper = np.array([5, 255, 255])

        mask_green = cv2.inRange(hsvFrame, green_lower, green_upper)

        mask_blue = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        mask_red = cv2.inRange(hsvFrame, red_lower, red_upper)

        # mask_final = cv2.bitwise_or(mask_blue, mask_red, mask_green)

        mask_partial = cv2.bitwise_or(mask_green, mask_blue)

        mask_final = cv2.bitwise_or(mask_partial, mask_red)

        target = cv2.bitwise_and(cv_image, cv_image, mask=mask_final)

        cv2.imwrite("test_final_mask.png", target)

        target_r = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)
        target_g = cv2.bitwise_and(cv_image, cv_image, mask=mask_green)
        target_b = cv2.bitwise_and(cv_image, cv_image, mask=mask_blue)

        cv2.imwrite("test_red_mask.png", target_r)
        cv2.imwrite("test_green_mask.png", target_g)
        cv2.imwrite("test_blue_mask.png", target_b)

        gray_r = cv2.cvtColor(target_r, cv2.COLOR_BGR2GRAY)
        blurred_r = cv2.GaussianBlur(gray_r, (5, 5), 0)
        thresh_r = cv2.threshold(blurred_r, 10, 255, cv2.THRESH_BINARY)[1]

        gray_g = cv2.cvtColor(target_g, cv2.COLOR_BGR2GRAY)
        blurred_g = cv2.GaussianBlur(gray_g, (5, 5), 0)
        thresh_g = cv2.threshold(blurred_g, 10, 255, cv2.THRESH_BINARY)[1]

        gray_b = cv2.cvtColor(target_b, cv2.COLOR_BGR2GRAY)
        blurred_b = cv2.GaussianBlur(gray_b, (5, 5), 0)
        thresh_b = cv2.threshold(blurred_b, 10, 255, cv2.THRESH_BINARY)[1]

        cv2.imwrite("test_red_thresh.png", thresh_r)
        cv2.imwrite("test_green_thresh.png", thresh_g)
        cv2.imwrite("test_blue_thresh.png", thresh_b)

        # cv2.imwrite("test_gray.png", gray)
        # cv2.imwrite("test_blurred.png", blurred)
        # cv2.imwrite("test_thresh.png", thresh)

        x_list = list()
        y_list = list()
        color_list = list()

        # RED
        cnts = cv2.findContours(thresh_r.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = str(int(M["m10"] / M["m00"]))
            cY = str(int(M["m01"] / M["m00"]))
            x_list.append(cX)
            y_list.append(cY)
            color_list.append('red')

        # GREEN
        cnts = cv2.findContours(thresh_g.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = str(int(M["m10"] / M["m00"]))
            cY = str(int(M["m01"] / M["m00"]))
            x_list.append(cX)
            y_list.append(cY)
            color_list.append('green')

        # BLUE
        cnts = cv2.findContours(thresh_b.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = str(int(M["m10"] / M["m00"]))
            cY = str(int(M["m01"] / M["m00"]))
            x_list.append(cX)
            y_list.append(cY)
            color_list.append('blue')

        centers_msg = centers()
        centers_msg.x_centers = x_list
        centers_msg.y_centers = y_list
        centers_msg.colors = color_list
        self.centers_pub.publish(centers_msg)


def main(args):
    ic = image_converter()
    rospy.init_node('img_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
