#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from priority_queue import *


class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        self.path = []
        rospy.init_node("path_planner")
        # self.getplan = rospy.Service("plan_path", GetPlan, self.plan_path)

        self.cspace_pub = rospy.Publisher("/path_planner/cspace", OccupancyGrid, queue_size=10)

        self.a = rospy.Service("a_star", GetPlan, self.plan_path)
        self.expanded_pub = rospy.Publisher("/path_planner/expanded", GridCells, queue_size=10)
        self.frontier_pub = rospy.Publisher("/path_planner/frontier", GridCells, queue_size=10)
        self.path_pub = rospy.Publisher("/path_planner/apath", Path, queue_size=10)
        # self.pose_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped)
        # rospy.Subscriber('/move_base_simple/frontier_goal', PoseStamped, self.get_goal)

        # TODO
        # Initialize the request counter


        rospy.loginfo("Path planner node ready")

        grid = PathPlanner.request_map()
        self.mapdata = grid
        self.cSpace = self.calc_cspace(grid, 3)
        # self.c_space_array = cSpace
        # grid_cells = PathPlanner.createGridcells(self.mapdata, cSpace.data)
        # self.cspace_pub.publish(grid_cells)

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        i = (y * mapdata.info.width) + x
        return int (i)

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
        distance = math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))
        return distance

    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        initMAPX = mapdata.info.origin.position.x
        initMAPY = mapdata.info.origin.position.y
        resol = mapdata.info.resolution
        WX = ((x + 0.5) * resol + initMAPX)
        WY = ((y + 0.5) * resol + initMAPY)

        pt = Point()
        pt.x = WX
        pt.y = WY
        pt.z = 0.0
        return pt

    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        WX = wp.x
        WY = wp.y
        resol = mapdata.info.resolution
        # -0.5 but coordinates to center
        gx = math.floor((WX - mapdata.info.origin.position.x) / resol - 0.5)
        gy = math.floor((WY - mapdata.info.origin.position.y) / resol - 0.5)
        return gx, gy

    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        p_array = []
        rospy.loginfo("Converting path to poses.")
        last_ori = quaternion_from_euler(0, 0, 0)
        last_ori = Quaternion(last_ori[0], last_ori[1],last_ori[2],last_ori[3])
        for i in range(len(path) - 1):
            msg = PoseStamped()
            msg.pose.position = PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1])
            last_ori = quaternion_from_euler(0, 0, PathPlanner.get_orientation(path[i], path[i+1]))
            last_ori = Quaternion(last_ori[0], last_ori[1],last_ori[2],last_ori[3])
            msg.pose.orientation = last_ori
            p_array.append(msg)

        last = PoseStamped()
        last.pose.position = PathPlanner.grid_to_world(mapdata, path[-1][0], path[-1][1])
        last.pose.orientation = last_ori
        p_array.append(last)
        return p_array
    
    @staticmethod
    def get_orientation(p1, p2): #calculate the global orientation of the robot travel from point1 to point2
        if p1[0] <p2[0]:
            if p1[1] <p2[1]:
                return math.pi / 4
            elif p1[1] ==p2[1]:
                return 0
            else:
                return -math.pi / 4
        if p1[0] == p2[0]:
            if p1[1] >p2[1]:
                return math.pi / 2
            else:
                return -math.pi / 2
        if p1[0] >p2[0]:
            if p1[1] <p2[1]:
                return 3 * math.pi / 4
            elif p1[1] == p2[1]:
                return math.pi
            else: 
                return -3 * math.pi / 4

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
        index = PathPlanner.grid_to_index(mapdata, x, y)
        return 0 <= index and index < (mapdata.info.width * mapdata.info.height) and mapdata.data[index] == 0

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        four_neigh = set()
        # if PathPlanner.is_cell_walkable(mapdata, x, y) == True:
        if (PathPlanner.is_cell_walkable(mapdata, x+1, y)):
            four_neigh |= {(x+1,y)}
        if (PathPlanner.is_cell_walkable(mapdata, x-1, y)):
            four_neigh |= {(x-1,y)}
        if (PathPlanner.is_cell_walkable(mapdata, x, y+1)):
            four_neigh |= {(x,y+1)}
        if (PathPlanner.is_cell_walkable(mapdata, x, y-1)):
            four_neigh |= {(x,y-1)}


        return four_neigh

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        uses neighbors_of_4
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        eight_neigh = set()
        # if PathPlanner.is_cell_walkable(mapdata, x, y) == True:
        eight_neigh |= PathPlanner.neighbors_of_4(mapdata,x,y)
        if (PathPlanner.is_cell_walkable(mapdata, x+1, y+1)):
            eight_neigh |= {(x+1,y+1)}
        if (PathPlanner.is_cell_walkable(mapdata, x-1, y-1)):
            eight_neigh |= {(x-1,y-1)}
        if (PathPlanner.is_cell_walkable(mapdata, x+1, y-1)):
            eight_neigh |= {(x+1,y-1)}
        if (PathPlanner.is_cell_walkable(mapdata, x-1, y+1)):
                eight_neigh |= {(x-1,y+1)} 

        return eight_neigh

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """

        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('dynamic_map')
        getMap = rospy.ServiceProxy('dynamic_map', GetMap)
        g = getMap().map

        return g

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")
        c_Space = OccupancyGrid()
        c_Space.header = mapdata.header
        c_Space.info = mapdata.info

        c_Space_data = list(mapdata.data)
        listOfPoints = []
        for x in range(mapdata.info.width):
            for y in range(mapdata.info.height):
                index = self.grid_to_index(mapdata,x,y)
                for j in range (padding +1) :
                    for k in range(padding +1):
                        if(mapdata.data[index] == 100):
                            c_Space_data[index] = 100
                            if ((x+j) > mapdata.info.width -1 or (x+j) < 0 or (y+k)> mapdata.info.height -1 or (y+k) <0):
                                pass
                            else:
                                index1 = self.grid_to_index(mapdata, (x+j), (y+k))
                                c_Space_data[index1] = 100
                                listOfPoints.append((x+j, y+k))

                            if ((x-j) > mapdata.info.width -1 or (x-j) < 0 or (y+k)> mapdata.info.height -1 or (y+k) <0):
                                pass
                            else:
                                index2 = self.grid_to_index(mapdata, (x-j), (y+k))
                                c_Space_data[index2] = 100
                                listOfPoints.append((x-j, y+k))                        

                            if ((x+j) > mapdata.info.width -1 or (x+j) < 0 or (y-k)> mapdata.info.height -1 or (y-k) <0):
                                pass
                            else:
                                index3 = self.grid_to_index(mapdata, (x+j), (y-k))
                                c_Space_data[index3] = 100
                                listOfPoints.append((x+j, y-k))

                            if ((x-j) > mapdata.info.width -1 or (x-j) < 0 or (y-k)> mapdata.info.height -1 or (y-k) <0):
                                pass
                            else:
                                index4 = self.grid_to_index(mapdata, (x-j), (y-k))
                                c_Space_data[index4] = 100
                                listOfPoints.append((x-j, y-k))
        c_Space.data=c_Space_data
        self.cspace_pub.publish(c_Space)
        rospy.loginfo("Publishing OccupancyGrid for C-Space")
        return c_Space

    @staticmethod
    def createGridcells(mapdata, listOfP):
        """
        Create a GridCellss message typy given a list of points in the grid coordinate
        """
        new_gridcells = GridCells()
        new_gridcells.header = mapdata.header
        new_gridcells.cell_width = mapdata.info.resolution
        new_gridcells.cell_height = mapdata.info.resolution
        new_gridcells.cells = []
        for p in listOfP:
            new_gridcells.cells.append(PathPlanner.grid_to_world(mapdata, p[0], p[1]))
        return new_gridcells
    
    def a_star(self, mapdata, start, goal):
        """
        Generate a path as a list of tuple from Start to goal using A*
        """

        print "Inside A star"
        rospy.loginfo("Generate path from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        if not PathPlanner.is_cell_walkable(mapdata, goal[0], goal[1]):
            rospy.logerr("not walkable goal")
            return[]
        #calculated from goal
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            frontier_msg = GridCells()
            frontier_cells = []
            for e in frontier.elements:
                frontier_cells.append(PathPlanner.grid_to_world(mapdata, e[1][0], e[1][1]))
            frontier_msg.header = mapdata.header
            frontier_msg.header.stamp = rospy.get_rostime()
            frontier_msg.cell_width = mapdata.info.resolution
            frontier_msg.cell_height = mapdata.info.resolution
            frontier_msg.cells = frontier_cells
            expanded_msg = GridCells()
            expanded_cells = []
            for e in cost_so_far:               
                expanded_cells.append(PathPlanner.grid_to_world(mapdata, e[0], e[1]))
            
            expanded_msg.header = mapdata.header
            expanded_msg.header.stamp = rospy.get_rostime()
            expanded_msg.cell_width = mapdata.info.resolution
            expanded_msg.cell_height = mapdata.info.resolution
            expanded_msg.cells = expanded_cells
            self.expanded_pub.publish(expanded_msg)
            rospy.sleep(0.01)

            current = frontier.get()

            #creates path
            if current == goal:
                entry = goal
                listOfCoord = []
                while entry != None:
                    listOfCoord.append(entry)
                    entry = came_from[entry]
                listOfCoord.reverse()
                self.expanded_pub.publish(PathPlanner.createGridcells(mapdata, listOfCoord))
                return listOfCoord
            
            for next in PathPlanner.neighbors_of_8(mapdata, current[0], current[1]):
                new_cost = cost_so_far[current] + 1 #assume cost to move each unit is 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(next[0], next[1], goal[0], goal[1])
                    frontier.put(next, priority)
                    came_from[next] = current

            
        return[] 

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        rospy.loginfo("Optimizing path")

        opt_path = []
        current_direction = (0, 0)
        last_direction = (0, 0)

        for i in range(len(path) -1):
            current_direction = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            if current_direction != last_direction:
                opt_path.append(path[i])
                last_direction = current_direction
            
        opt_path.append(path[-1]) #add the last coordinate back

        return opt_path

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        rospy.loginfo("Returning a Path message")
        pathObj = Path()
        pathObj.header.frame_id = '/map'
        pathObj.poses = PathPlanner.path_to_poses(mapdata, path)
        return pathObj

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        # Request the map
        # In case of error, return an empty path
        mapdata = PathPlanner.request_map()

        if mapdata is None:
            return Path()
        # Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 3)
        # Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        
        path = self.a_star(cspacedata, start, goal) #, self.c_space_array, self.frontier, self.expanded)
   
        # Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        # print waypoints
        waypoints.remove(waypoints[0])
        # print waypoints

        self.path_pub.publish(self.path_to_message(cspacedata, waypoints))
        # Return a Path message
        return self.path_to_message(cspacedata, waypoints)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()


if __name__ == '__main__':
    turtle10 = PathPlanner()
    turtle10.run()
    # map = turtle10.request_map()
    # cspace = turtle10.calc_cspace(map,1)
    # path = turtle10.a_star(cspace, (16,16), (21,29))
    # opPath = PathPlanner.optimize_path(path)
    # turtle10.expanded_pub.publish(PathPlanner.createGridcells(map, path))
    # turtle10.frontier_pub.publish(PathPlanner.createGridcells(map, opPath)
