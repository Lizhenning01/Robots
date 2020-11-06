#!/usr/bin/env python2

import numpy as np
import os

import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from priority_queue import *
from path_planner import *
from client import *

# TODO: update_map, save_map

# 1. use current scan data to calculate the current cspace
# 2. use update_frontier to add all accessible but unexplored points to the frontier
# 3. find closest unexplored point, use A* to navigate there (maybe delete last waypoint to avoid running into walls)
# 4. repeat until 90+% of map is explored
# 5. return to starting location


class FrontierExploration:

    def __init__(self):
        rospy.loginfo("Starting FrontierExploration node...")
        rospy.init_node("frontier_exploration")

        # store OccupancyGrid
        self.mapdata = PathPlanner.request_map()
        self.cspace = self.calc_cspace(self.mapdata, 3)
        self.unexplored_frontier = []

        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0
        self.start_location = 0

        # rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.update_odometry)
        rospy.Subscriber('odom', Odometry, self.update_odometry)
        # rospy.Subscriber('/path_planner/cspace', OccupancyGrid, self.update_cspace)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.frontier_goal_pub = rospy.Publisher('/move_base_simple/frontier_goal', PoseStamped, queue_size=10)
        self.goal_viz_pub = rospy.Publisher('/goal_location', GridCells, queue_size=10)
        self.frontier_pub = rospy.Publisher('/explore_frontier', GridCells, queue_size=10)

        rospy.loginfo("FrontierExploration node started.")
        rospy.sleep(3)

        # loop until map is mostly revealed
        while (len(self.unexplored_frontier) == 0) or (len(self.unexplored_frontier) > 10):
            # store frontier as list of points
            self.mapdata = PathPlanner.request_map()
            self.cspace = self.calc_cspace(self.mapdata, 3)
            self.unexplored_frontier = []
            self.update_frontier()

            frontier_msg = GridCells()
            frontier_cells = []
            for e in range(len(self.unexplored_frontier)):
                x = e % self.cspace.info.width
                y = e / self.cspace.info.height
                p = Point()
                p.x = x
                p.y = y
                frontier_cells.append(p)
            frontier_msg.header = self.cspace.header
            frontier_msg.header.stamp = rospy.get_rostime()
            frontier_msg.cell_width = self.cspace.info.resolution
            frontier_msg.cell_height = self.cspace.info.resolution
            frontier_msg.cells = frontier_cells
            self.frontier_pub.publish(frontier_msg)

            nearest = self.find_nearest_frontier()
            rospy.loginfo("Going to frontier: " + str(nearest))
            self.create_frontier_goal_msg(nearest)
            rospy.sleep(20)

        # save map
        self.save_map()

        # return to start
        msg = PoseStamped()
        msg.pose = self.start_location
        msg.header.frame_id = 'map'
        self.goal_pub.publish(msg)

        # after, robot will continue to respond to new /move_base_simple/goal messages from rviz

    def create_goal_msg(self, point):
        msg = PoseStamped()
        msg.pose.position = point
        self.goal_pub.publish(msg)

    def create_frontier_goal_msg(self, point):
        msg = PoseStamped()
        msg.pose.position = point
        self.frontier_goal_pub.publish(msg)

    def update_map(self):
        ## request map
        self.mapdata = PathPlanner.request_map()

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")
        #cSpace = list(mapdata.data)
        c_Space = OccupancyGrid()
        c_Space.header = mapdata.header
        c_Space.info = mapdata.info

        # c_Space_data = [0] * (mapdata.info.width * mapdata.info.height)
        c_Space_data = list(mapdata.data)
        listOfPoints = []
        for x in range(mapdata.info.width):
            for y in range(mapdata.info.height):
                index = PathPlanner.grid_to_index(mapdata,x,y)
                for j in range (padding +1) :
                    for k in range(padding +1):
                        if(mapdata.data[index] == 100):
                            c_Space_data[index] = 100
                            if ((x+j) > mapdata.info.width -1 or (x+j) < 0 or (y+k)> mapdata.info.height -1 or (y+k) <0):
                                pass
                            else:
                                index1 = PathPlanner.grid_to_index(mapdata, (x+j), (y+k))
                                c_Space_data[index1] = 100
                                listOfPoints.append((x+j, y+k))

                            if ((x-j) > mapdata.info.width -1 or (x-j) < 0 or (y+k)> mapdata.info.height -1 or (y+k) <0):
                                pass
                            else:
                                index2 = PathPlanner.grid_to_index(mapdata, (x-j), (y+k))
                                c_Space_data[index2] = 100
                                listOfPoints.append((x-j, y+k))

                            if ((x+j) > mapdata.info.width -1 or (x+j) < 0 or (y-k)> mapdata.info.height -1 or (y-k) <0):
                                pass
                            else:
                                index3 = PathPlanner.grid_to_index(mapdata, (x+j), (y-k))
                                c_Space_data[index3] = 100
                                listOfPoints.append((x+j, y-k))

                            if ((x-j) > mapdata.info.width -1 or (x-j) < 0 or (y-k)> mapdata.info.height -1 or (y-k) <0):
                                pass
                            else:
                                index4 = PathPlanner.grid_to_index(mapdata, (x-j), (y-k))
                                c_Space_data[index4] = 100
                                listOfPoints.append((x-j, y-k))
        c_Space.data=c_Space_data
        return c_Space

    def save_map(self):
        # Changes the directory to maps dir
        # os.chdir("/maps")
        # Saves the map by writing on system terminal
        os.system("rosrun map_server map_saver -f mymap")

    def update_frontier(self):
        rospy.loginfo("Updating accessible frontier...")
        ## cycle through all spaces in occupancy grid
        for i in range(len(self.cspace.data)):
            ## check if space is == -1 (means unexplored)
            if self.cspace.data[i] == -1:
                ## check if there is a known walkable space nearby
                x, y = i % self.cspace.info.width, i / self.cspace.info.height
                neighbors = PathPlanner.neighbors_of_8(self.cspace, x, y)
                if len(neighbors) != 0:
                    ## append to unexplored_frontier
                    p = Point()
                    p.x = x
                    p.y = y
                    self.unexplored_frontier.append(p)
        ## cycle through unexplored_frontier and remove all explored nodes (!== -1)

        rospy.loginfo("Accessible frontier updated.")
        return

    def find_nearest_frontier(self):
        rospy.loginfo("Locating nearest accessible frontier...")
        nearest_unexplored_frontier = -1
        nearest_distance = 9999

        # transform OccupancyGrid into GridCells
        # cycle through each item in the unexplored_frontier
        for i in range(len(self.unexplored_frontier)):
            # calculate euclidian distance for each point, keep track of shortest distance
            world_cell = PathPlanner.grid_to_world(self.cspace, self.unexplored_frontier[i].x, self.unexplored_frontier[i].y)
            x1 = self.px
            x2 = world_cell.x
            y1 = self.py
            y2 = world_cell.y
            dist = PathPlanner.euclidean_distance(x1, x2, y1, y2)
            if dist < nearest_distance:
                nearest_distance = dist
                nearest_unexplored_frontier = self.unexplored_frontier[i]

        rospy.loginfo("Found accessible frontier.")
        # find walkable neighbor to make A* happy
        nearest_unexplored_frontier = PathPlanner.neighbors_of_8(self.cspace, nearest_unexplored_frontier.x, nearest_unexplored_frontier.y)
        # return point to use A* to go to it
        nearest_unexplored_frontier = nearest_unexplored_frontier.pop()
        nearest = PathPlanner.grid_to_world(self.cspace,nearest_unexplored_frontier[0], nearest_unexplored_frontier[0])
        msg = GridCells()
        msg.header.frame_id = 'map'
        msg.cell_width = self.cspace.info.resolution
        msg.cell_height = self.cspace.info.resolution
        msg.cells = [nearest]
        self.goal_viz_pub.publish(msg)
        return nearest

    def update_odometry(self, msg):

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_ori = msg.pose.pose.orientation
        quat_list = [quat_ori.x, quat_ori.y, quat_ori.z, quat_ori.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

        if self.start_location == 0:
            p = Pose()
            p.position.x = self.px
            p.position.y = self.py
            self.start_location = p

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    lab4_1 = FrontierExploration()
    lab4_1.run()
