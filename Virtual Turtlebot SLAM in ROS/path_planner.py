#!/usr/bin/env python
import copy
import math

import numpy as np
import rospy
import priority_queue as pq
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
import tf.transformations
import rosservice

# TODO: A* is not properly accounting for obstacles (it does not see them)

# TODO: robot does not drive to correct location and has misplaced obstacles, this is due to problems in map
#  conversion (at least). Current major issues with frame transformations somewhere between the Path (which is correct)
#  and the coordinates received by the drive methods. There are intermediate steps like publishing the A*
#  frontier that complicate this process

class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node('path_planner')
        # rospy.init_node('path_planner', anonymous=True)

        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        # service_list=rosservice.get_service_list()
        # rospy.loginfo("running services: %s" % service_list)
        rospy.Service('plan_path', GetPlan, self.plan_path)

        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace_pub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)

        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.frontier_pub = rospy.Publisher('/frontier_gridcells', GridCells, queue_size=1)
        self.expanded_pub = rospy.Publisher('/expanded_gridcells', GridCells, queue_size=1)
        self.path_grid_pub = rospy.Publisher('/path_gridcells', GridCells, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint_gridcells', GridCells, queue_size=1)

        self.path_pub = rospy.Publisher('/path_Path', Path, queue_size=1)

        ## Initialize the request counter
        request_counter = 0

        # Get map for c-space setup currently pull the map right before we do any pathplanning
        # self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.calc_cspace)
        mapdata = PathPlanner.request_map()
        self.cspacedata = self.calc_cspace(mapdata, 1)

        ## Sleep to allow roscore to do some housekeeping
        # rospy.sleep(0.5)
        rospy.loginfo("Path planner node ready.")

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """

        index = y * mapdata.info.width + x

        return index

    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        point = Point()
        point.x = x * mapdata.info.resolution + mapdata.info.origin.position.x
        point.y = y * mapdata.info.resolution + mapdata.info.origin.position.y
        point.z = 0
        return point

    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        resolution = mapdata.info.resolution
        x_i = int((wp.x - mapdata.info.origin.position.x) / resolution)
        y_i = int((wp.y - mapdata.info.origin.position.y) / resolution)
        return [(x_i, y_i)]

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return dist

    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        poses = []
        for i in range(len(path)-1):
            current = path[i]
            next = path[i+1]

            current = PathPlanner.grid_to_world(mapdata, current[0], current[1])
            next = PathPlanner.grid_to_world(mapdata, next[0], next[1])

            yaw = math.atan2(next.x - current.x, next.y - current.y)
            current_pose = Pose()
            current_pose.position = current
            current_pose.orientation.x = 0
            current_pose.orientation.y = 0
            current_pose.orientation.z = 0
            current_pose.orientation.w = 0

            current_posestamped = PoseStamped()
            current_posestamped.header.frame_id = "odom"
            current_posestamped.header.stamp = rospy.Time.now()

            current_posestamped.pose = current_pose

            poses.append(current_posestamped)
        current_pose = Pose()
        current_pose.position = next
        current_pose.orientation.x = 0
        current_pose.orientation.y = 0
        current_pose.orientation.z = 0
        current_pose.orientation.w = 0

        current_posestamped = PoseStamped()
        current_posestamped.header.frame_id = "odom"
        current_posestamped.header.stamp = rospy.Time.now()

        current_posestamped.pose = current_pose

        poses.append(current_posestamped)


        # rospy.loginfo("Path: " + str(poses))
        path_msg = Path()
        path_msg.poses = poses
        path_msg.header.frame_id = 'map'
        return path_msg

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        # wp = Point()
        # wp.x = x
        # wp.y = y
        # point = PathPlanner.world_to_grid(mapdata, wp)[0]
        # in_boundaries = (point[0] >=0 and point[0] < mapdata.info.width) and \
        #                 (point[0] >=0 and point[0] < mapdata.info.width)
        in_boundaries = (x >=0 and x < mapdata.info.width) and \
                        (y >=0 and y < mapdata.info.height)
        if in_boundaries:
            index = PathPlanner.grid_to_index(mapdata, x, y)
            # rospy.loginfo("Index: " + str(index))
            # rospy.loginfo("Index value: " + str(mapdata.data[index]))
            occupancy = mapdata.data[index]
            free = occupancy != 100
        walkable = in_boundaries and free
        return walkable

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        neighbors = []
        offset = [-1, 1]
        for off in offset:
            if PathPlanner.is_cell_walkable(mapdata, x + off, y):
                newNeighbor = (x + off, y)
                neighbors.append(newNeighbor)
            if PathPlanner.is_cell_walkable(mapdata, x, y + off):
                newNeighbor = (x, y + off)
                neighbors.append(newNeighbor)
        return neighbors

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        neighbors = PathPlanner.neighbors_of_4(mapdata, x, y)
        neigborsDiagonals = []
        offset = [-1, 1]
        for off1 in offset:
            for off2 in offset:
                if (x + off1, y) in neighbors and \
                        (x, y + off2) in neighbors and \
                        PathPlanner.is_cell_walkable(mapdata, x + off1, y + off2):
                    neigborsDiagonals.append((x + off1, y + off2))
        for i in range(len(neigborsDiagonals)):
            neighbors.append(neigborsDiagonals[i])
        return neighbors

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map")

        # rospy.wait_for_service('map')
        try:
            Imported = rospy.ServiceProxy('static_map', GetMap)
            resp1 = Imported()

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None
        rospy.loginfo("Got map")
        return resp1.map

    def get_og_val(self, i, j, map_og):
        return map_og.data[PathPlanner.grid_to_index(map_og,i,j)]

    def set_og_val(self, i, j, val, map):
        map.data = list(map.data)
        if self.is_cell_walkable(map, i, j):
            map.data[map.info.width * i + j] = val
        map.data = tuple(map.data)
        return map

    def gen_cspace(self, mapdata, padding):
        cspace_og = copy.deepcopy(mapdata)
        gridcell_list=[]
        # self.map_res = self.map.info.resolution
        # self.map_width = self.map.info.width
        # self.map_height = map.info.height

        for i in range(0, mapdata.info.height):
            for j in range(0, mapdata.info.width):

                # check if map cell is a wall
                if self.get_og_val(i, j, mapdata) == 100:
                    p = PathPlanner.grid_to_world(mapdata, i, j)
                    if p not in gridcell_list:
                        gridcell_list.append(p)

                    # set surrounding 8 cells in cspace to walls
                    for i_offset in range(-padding, padding + 1):
                        for j_offset in range(-padding, padding + 1):
                            cspace_og = self.set_og_val(i + i_offset, j + j_offset, 100, cspace_og)
                            p = PathPlanner.grid_to_world(mapdata, i + i_offset, j + j_offset)
                            if p not in gridcell_list:
                                gridcell_list.append(p)
        self.cspace = cspace_og
        return gridcell_list

    def calc_cspace(self, mapdata, padding=1):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        # all of this is done in gen_cspace
        expanded_obstacles = self.gen_cspace(mapdata, padding)

        # Create a GridCells message and publish it to display obstacles on Rviz
        gc_msg = GridCells()
        gc_msg.cell_height = mapdata.info.resolution
        gc_msg.cell_width = mapdata.info.resolution
        gc_msg.cells = expanded_obstacles
        gc_msg.header.frame_id = 'odom'
        self.cspace_pub.publish(gc_msg)

        new_occupancy_grid = []

        for i in range(mapdata.info.width):
            for j in range(mapdata.info.height):
                new_occupancy_grid.append(0)

        # for i in range(len(expanded_obstacles)):
        #     p = PathPlanner.world_to_grid(mapdata, expanded_obstacles[i])
        #     index = PathPlanner.grid_to_index(mapdata, -p[0][0], -p[0][1])
        #     new_occupancy_grid[index] = 100

        ## Return the C-space
        new_cspace = mapdata
        new_cspace.data = new_occupancy_grid
        return new_cspace

    def a_star(self, mapdata, start, goal):
        # why are these values coming in negative??
        start = [start[0][0], start[0][1]]
        goal = [goal[0][0], goal[0][1]]
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        rospy.loginfo("Robot should stop at " +
                      str(PathPlanner.grid_to_world(mapdata, goal[0], goal[1])) + " in world coords.")

        node_start = (start[0], start[1])
        node_goal = (goal[0], goal[1])

        # create list of unoccupied coordinates
        open_nodes = []
        for i in range(len(mapdata.data)):
            if mapdata.data[i] is not 100:
                x = i % mapdata.info.width
                y = i / mapdata.info.height
                open_nodes.append([x, y])

        # start at start node, loop until path ends at goal node
        current_node = node_start
        path = []
        h_low = 999999
        frontier_cells = []
        expanded_cells = []
        next_node = 0
        prev_node = 0
        while current_node != node_goal:
            # look at all neighbors surrounding current node, calculate h for each node, then add lowest to path
            neighbors = self.neighbors_of_8(mapdata, current_node[0], current_node[1])
            for i in range(len(neighbors)):
                frontier_cells.append(PathPlanner.grid_to_world(mapdata, neighbors[i][0], neighbors[i][1]))

            gc_msg = GridCells()
            gc_msg.cell_height = .3
            gc_msg.cell_width = .3
            gc_msg.cells = frontier_cells
            gc_msg.header.frame_id = 'odom'
            self.frontier_pub.publish(gc_msg)

            # cycle through each neighbor, find lowest h value
            for i in range(len(neighbors)):
                h = PathPlanner.euclidean_distance(neighbors[i][0], neighbors[i][1], node_goal[0], node_goal[1])
                gc_msg = GridCells()
                gc_msg.cell_height = .3
                gc_msg.cell_width = .3
                expanded_cells.append(Point())
                expanded_cells[i].x = neighbors[i][0]
                expanded_cells[i].y = neighbors[i][1]
                gc_msg.cells = expanded_cells
                gc_msg.header.frame_id = 'odom'
                self.expanded_pub.publish(gc_msg)

                if h_low > h:
                    h_low = h
                    next_node = neighbors[i]
                    current_node = neighbors[i]

            ## if the robot is in a corner and can't find a closer node next to it, start expanding neighbors
            # check if the "next node" is the same as current one (so it's stuck)
            # while current_node == prev_node:
            #     for k in range(len(neighbors)):
            #         # find neighbors of neighbors
            #         new_neighbors = self.neighbors_of_8(mapdata, neighbors[i][0], neighbors[i][1])
            #         for i in range(len(new_neighbors)):
            #             # add new entries to neighbors and frontier if not already present
            #             trimmed_new_neighbors = []
            #             if not new_neighbors[i] in neighbors:
            #                 new_frontier = new_neighbors[i]
            #                 neighbors.append(new_frontier)
            #                 frontier_cells.append(PathPlanner.grid_to_world(mapdata, new_frontier[0], new_frontier[1]))
            #                 trimmed_new_neighbors.append(new_frontier)
            #
            #         # cycle through each new neighbor, find lowest h value
            #         for i in range(len(trimmed_new_neighbors)):
            #             h = PathPlanner.euclidean_distance(trimmed_new_neighbors[i][0], trimmed_new_neighbors[i][1],
            #                                                node_goal[0], node_goal[1])
            #             if h_low > h:
            #                 h_low = h
            #                 next_node = new_neighbors[i]
            #                 current_node = new_neighbors[i]
            #
            #         # publish frontier
            #         gc_msg = GridCells()
            #         gc_msg.cell_height = .3
            #         gc_msg.cell_width = .3
            #         gc_msg.cells = frontier_cells
            #         gc_msg.header.frame_id = 'odom'
            #         self.frontier_pub.publish(gc_msg)

            if prev_node == current_node:
                return [None]
            prev_node = current_node
            path.append(next_node)

        rospy.loginfo(path)
        #
        # frontier_cells = []
        # # close/decolor frontier
        # gc_msg = GridCells()
        # gc_msg.cell_height = .3
        # gc_msg.cell_width = .3
        # gc_msg.cells = frontier_cells
        # gc_msg.header.frame_id = 'odom'
        # self.frontier_pub.publish(gc_msg)

        # color path on map
        grid_path = []
        for i in range(len(path)):
            grid_path.append(PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1]))
        gc_msg = GridCells()
        gc_msg.cell_height = .3
        gc_msg.cell_width = .3
        gc_msg.cells = grid_path
        gc_msg.header.frame_id = 'odom'
        self.expanded_pub.publish(gc_msg)

        return path

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        newPath=[]
        rospy.loginfo("Optimizing path")
        #if there are any intermediate points
        if len(path)>=3:
            #establish reference (waypoint since last turn)
            reference = path[0]
            #add to new path
            newPath.append(reference)

            #get next point and establish the angle between them
            next = path[1]
            angle = math.atan2(next[1]-reference[1],next[0]-reference[0])

            for i in range(2,len(path)):
                next=path[i]
                #angle between current node and node since last angle change
                next_angle=math.atan2(next[1]-reference[1],next[0]-reference[0])

                #if there is an angle change or goal is reached
                if angle != next_angle or i==(len(path)-1):
                    #establish new reference as previous point
                    reference=path[i-1]
                    newPath.append(reference)
        else:
            newPath=path

        return newPath

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        posestamparray = self.path_to_poses(mapdata, path).poses

        ## Convert Poses back to world coords
        # world_poses = []
        # for i in range(len(posestamparray)):
        #     pose = PoseStamped()
            # pose.pose.position = PathPlanner.grid_to_world(self.cspacedata, posestamparray[i].pose.position.x,
            #                                              posestamparray[i].pose.position.y)
            # world_poses.append(pose)

        pathmessage = Path()
        pathmessage.poses = posestamparray
        pathmessage.header.frame_id = 'odom'
        return pathmessage

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """
        ## Request the map
        ## In case of error, return an empty path
        rospy.loginfo("Starting plan_path service...")
        # rospy.wait_for_service('/map')
        # mapdata = PathPlanner.request_map()
        # if mapdata is None:
        #     return Path()

        ## Calculate the C-space and publish it
        try:
            start = PathPlanner.world_to_grid(self.cspace, msg.start.pose.position)
        except rospy.ServiceException, e:
            print "Path planning initialization failed: %s" % e
            print "Node likely still starting up."
            return None
        goal = PathPlanner.world_to_grid(self.cspace, msg.goal.pose.position)

        # check if goal is walkable
        if not self.is_cell_walkable(self.cspace, goal[0][0], goal[0][1]):
            rospy.logerr("Destination cell unreachable, cancelling pathplanning.")
            return

        ## Execute A*
        path_poses = self.a_star(self.cspace, start, goal)

        ## Optimize waypoints
        # path_poses = PathPlanner.optimize_path(path)

        ## Return a Path message
        path_msg = self.path_to_message(self.cspace, path_poses)
        rospy.loginfo("Publishing Path message")
        self.path_pub.publish(path_msg)

        # waypoints_msg = []
        # for i in range(len(path_poses)):
        #     wpp = PathPlanner.grid_to_world(self.cspacedata, path_poses[i][0], path_poses[i][1])
        #     waypoints_msg.append([wpp.x, wpp.y])
        # waypoints_msg = self.path_to_message(self.cspacedata, waypoints_msg)
        # return waypoints_msg

        return path_msg

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

    @staticmethod
    def point_to_tuple(point):
        x = point.x
        y = point.y
        return (x, y)


if __name__ == '__main__':
    PathPlanner().run()
