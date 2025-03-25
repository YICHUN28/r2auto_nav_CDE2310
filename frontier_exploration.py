#!/usr/bin/env python3

import os
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import threading
import subprocess
import numpy as np
from typing import Union
from path_planner import PathPlanner
from frontier_search import FrontierSearch
from nav_msgs.msg import OccupancyGrid, Path, GridCells, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool
from custom_interfaces.msg import FrontierList
import rclpy.time
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from time import sleep


class FrontierExploration(Node):
    
    def __init__(self):
        super().__init__("frontier_exploration")
#Class constructor

        # Set if in debug mode
        self.is_in_debug_mode = True

        # Publishers
        self.pure_pursuit_pub = self.create_publisher(Path, "/pure_pursuit/path", 10)

#        if self.is_in_debug_mode:
        self.frontier_cells_pub = self.create_publisher(GridCells, "/frontier_exploration/frontier_cells", 10)
        self.start_pub = self.create_publisher(GridCells, "/frontier_exploration/start", 10)
        self.goal_pub = self.create_publisher(GridCells, "/frontier_exploration/goal", 10)
        self.cspace_pub = self.create_publisher(OccupancyGrid, "/cspace", 10)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'cost_map', 10,)

        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            "odom",
            self.get_odom,
            10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "map",
            self.update_map,
            qos_profile_sensor_data
        )
        self.navsub = self.create_subscription(Bool,"/navigatingyesno",self.update_nav,10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(2.0, self.check_transform)


        #self.lock = threading.Lock()
        self.pose = None
        self.map = None
        self.is_navigating = Bool()
        self.is_navigating.data = False

        self.NUM_EXPLORE_FAILS_BEFORE_FINISH = 30
        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False

        self.spin_thread = threading.Thread(target=rclpy.spin,args=(self,))
        self.spin_thread.daemon = True
        self.spin_thread.start()    

    def check_transform(self):
        if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
            self.timer = self.create_timer(2.0, self.get_odom)  # Start main loop
        else:
            self.get_logger().info("warning")


    
    def get_odom(self, msg: Union[Odometry, None] = None):
        try:
            # transform_map_to_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(), Duration(seconds=1))
            # transform_odom_to_base = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            # transform_base_to_map = self.tf_buffer.transform(transform_odom_to_base, "map")
            transform_base_to_map = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), Duration(seconds=1))
            trans = transform_base_to_map.transform.translation
            rot = transform_base_to_map.transform.rotation

            self.pose = Pose(
                position=Point(x=trans.x, y=trans.y),
                orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
            )

            #print(self.pose.position)

        except tf2_ros.TransformException or tf2_ros.LookupException:
            print("didnt work")


    def update_map(self, msg: OccupancyGrid):
        #Updates the current map.
        #his method is a callback bound to a Subscriber.
        #param msg [OccupancyGrid] The current map information.
        self.map = msg

    def update_nav(self,msg:Bool):
        self.is_navigating = msg

    def save_map(self):
        # Get the path of the current package
        package_path = get_package_share_directory("amazeing")

        # Construct the path to the map
        map_path = os.path.join(package_path, "map/map")
        if not os.path.exists(os.path.dirname(map_path)):
            os.makedirs(os.path.dirname(map_path))

        # Run map_saver
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", map_path])

        self.get_odom()

        if self.pose is None:
            self.get_logger().info("Failed to get pose")
            return

        # Save the robot's position and orientation
        position = self.pose.position
        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        with open(os.path.join(package_path, "map/pose.txt"), "w") as f:
            f.write(f"{position.x} {position.y} {position.z} {yaw} {pitch} {roll}\n")

    @staticmethod
    def get_top_frontiers(frontiers, n):
        # Sort the frontiers by size in descending order
        sorted_frontiers = sorted(
            frontiers, key=lambda frontier: frontier.size, reverse=True
        )

        # Return the top n frontiers
        return sorted_frontiers[:n]

    def publish_cost_map(self, mapdata: OccupancyGrid, cost_map: np.ndarray):
        # Create an OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = mapdata.info.resolution
        grid.info.width = cost_map.shape[1]
        grid.info.height = cost_map.shape[0]
        grid.info.origin = mapdata.info.origin

        # Normalize the cost map to the range [0, 100] and convert it to integers
        cost_map_normalized = (cost_map / np.max(cost_map) * 100).astype(np.int8)

        # Flatten the cost map and convert it to a list
        grid.data = cost_map_normalized.flatten().tolist()

        # Publish the OccupancyGrid message
        self.publisher_.publish(grid)

    def check_if_finished_exploring(self):
        # Publish empty path to stop the robot
        self.pure_pursuit_pub.publish(Path())

        # If no frontiers or paths are found for a certain number of times, finish exploring
        if (
            self.no_frontiers_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
            or self.no_path_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
        ):
            self.get_logger().info("Done exploring!")
            # self.save_map()
            self.get_logger().info("Saved map")
            self.is_finished_exploring = True

    def explore_frontier(self, frontier_list: FrontierList):
        # If finished exploring, no pose, no map, or no frontier list, return
        if self.is_finished_exploring or self.pose is None or self.map is None:
            return

        frontiers = frontier_list.frontiers

        # If no frontiers are found, check if finished exploring
        if not frontiers:
            self.get_logger().info("No frontiers")
            self.no_frontiers_found_counter += 1
            self.check_if_finished_exploring()
            return
        else:
            self.no_frontiers_found_counter = 0

        A_STAR_COST_WEIGHT = 10.0
        FRONTIER_SIZE_COST_WEIGHT = 1.0

        # Calculate the C-space
        cspace, cspace_cells = PathPlanner.calc_cspace(self.map, self.is_in_debug_mode)
        # if cspace_cells is not None:
        self.cspace_pub.publish(cspace)

        # Calculate the cost map
        cost_map = PathPlanner.calc_cost_map(self.map)
        if self.is_in_debug_mode:
            self.publish_cost_map(self.map, cost_map)

        # Get the start
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Execute A* for every frontier
        lowest_cost = float("inf")
        second_lowest_cost = float("inf")
        best_path = None
        second_best_path = None


        # Check only the top frontiers in terms of size
        MAX_NUM_FRONTIERS_TO_CHECK = 8
        top_frontiers = FrontierExploration.get_top_frontiers(
            frontiers, MAX_NUM_FRONTIERS_TO_CHECK
        )

        starts = []
        goals = []
        paths = []

        # Log how many frontiers are being explored
        self.get_logger().info(f"Exploring {len(top_frontiers)} frontiers")

        for frontier in top_frontiers:
            # Get goal coordinates
            print("sent to planner")
            goal = PathPlanner.world_to_grid(self.map, frontier.centroid)
            # Execute A*
            path, a_star_cost, start, goal = PathPlanner.a_star(
                cspace, cost_map, start, goal
            )
            print(path)

            # If in debug mode, append start and goal
            if self.is_in_debug_mode:
                starts.append(start)
                goals.append(goal)

            if path is None or a_star_cost is None:
                continue

            # Calculate cost
            cost = (A_STAR_COST_WEIGHT * a_star_cost) + (
                FRONTIER_SIZE_COST_WEIGHT / frontier.size
            )


            paths.append(path)
            print(paths)
            #Update best path
            # if cost < lowest_cost:
            #     lowest_cost = cost
            #     best_path = path
                # if second_lowest_cost > lowest_cost and second_lowest_cost < cost:
                #     second_best_path = path
                #     second_lowest_cost = cost
                # if cost > lowest_cost and cost 
        try: 

            if paths: 
                for i in range(len(paths)):
                    print('entered paths loop')
                    best_path = paths[i]
                        
                    print("best path is", best_path)
                    # If in debug mode, publish the start and goal
                    if self.is_in_debug_mode:
                        print("published start and goal")
                        print(starts, goals)
                        self.start_pub.publish(PathPlanner.get_grid_cells(self.map, starts))
                        self.goal_pub.publish(PathPlanner.get_grid_cells(self.map, goals))

                    # If a path was found, publish it
                    if best_path:
                        #self.get_logger().info(f"Found best path with cost {lowest_cost}")
                        start = best_path[0]
                        path = PathPlanner.path_to_message(self.map, best_path)   #converts best_path which is a list of tuples of coordinates into a ros path message
                        self.pure_pursuit_pub.publish(path)
                        print("path published")
                        print(self.is_navigating)
                        self.no_path_found_counter = 0
                    # If no path was found, check if finished exploring

                    sleep(1)
                    print(self.is_navigating)

                    while self.is_navigating.data == True:
                        print("robot is moving FORWARDS")
                        sleep(0.1) 
                    print(self.is_navigating)
                        
                        
                    best_path.reverse()

                    print("returning back")
                    print(best_path)
                    path = PathPlanner.path_to_message(self.map, best_path)   #converts best_path which is a list of tuples of coordinates into a ros path message
                    self.pure_pursuit_pub.publish(path)
   

                    sleep(1)

                    while self.is_navigating == True:
                        print("robot is moving IN REVERSE")
                        

            else:

                self.get_logger().info("No paths found")
                self.no_path_found_counter += 1
                self.check_if_finished_exploring()
        except Exception as e:
            return



    def run(self):
        try:
            # print("im working")
            if self.is_navigating.data == True:
                # print("robot is moving")
                return
                
            # self.check_transform()
            # if self.pose == None:
            #     print('found issue')
            # if self.pose != None:
            #     print("lesgo?")

            if self.pose is None or self.map is None:
                #rclpy.spin_once(self)
                return
            # Get the start position of the robot

            start = PathPlanner.world_to_grid(self.map, self.pose.position)
            if start != None:
                print(start)
            # Get frontiers
            frontier_list, frontier_cells = FrontierSearch.search(self.map, start, self.is_in_debug_mode)

            if frontier_list is None:
                #rclpy.spin_once(self)
                return
            
        # Publish frontier cells if in debug mode
            if self.is_in_debug_mode:
                self.frontier_cells_pub.publish(PathPlanner.get_grid_cells(self.map, frontier_cells))
            
            self.explore_frontier(frontier_list)
            sleep(0.1)
            # rclpy.spin_once(self)
        except KeyboardInterrupt:
            self.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    frontier_explorer = FrontierExploration()
    while rclpy.ok():
        frontier_explorer.run()
    #frontier_explorer.check_transform()
    # rclpy.spin(frontier_explorer)

