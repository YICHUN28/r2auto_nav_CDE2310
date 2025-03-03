#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from path_planner import PathPlanner
import rclpy.time
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Path, Odometry, GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3, Pose, Quaternion
from tf_transformations import euler_from_quaternion
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from typing import Union
from rclpy.qos import QoSProfile, ReliabilityPolicy
from time import sleep



class PurePursuit(Node):
    def __init__(self):
        """
        Class constructor
        """
        super().__init__("pure_pursuit")
        # Set if in debug mode
        self.is_in_debug_mode = True

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel", 10)
        self.lookahead_pub = self.create_publisher(PointStamped,"/pure_pursuit/lookahead", 10)
        self.fov_cells_pub = self.create_publisher(GridCells,"/pure_pursuit/fov_cells", 100)
        self.close_wall_cells_pub = self.create_publisher(GridCells, "/pure_pursuit/close_wall_cells", 100)

        # Subscribers
        self.odom_sub = self.create_subscription(
             Odometry,
             "/odom",
             self.update_odometry,
             10
            )
        self.map_sub = self.create_subscription(
             OccupancyGrid,
             "/map",
             self.update_map,
             qos_profile_sensor_data
            )
        #self.pure_pursuit_sub = self.create_subscription(Path, "/pure_pursuit/path", self.pathget, 100)
        self.pure_pursuit_toggle_sub = self.create_subscription(Bool, "/pure_pursuit/enabled", self.update_enabled,10)

        # Pure pursuit parameters
        self.LOOKAHEAD_DISTANCE = 0.10  # m
        self.WHEEL_BASE = 0.16  # m
        self.MAX_DRIVE_SPEED = 0.1  # m/s
        self.MAX_TURN_SPEED = 1.25  # rad/s
        self.TURN_SPEED_KP = 1.25
        self.DISTANCE_TOLERANCE = 0.1  # m

        # Obstacle avoidance parameters
        self.OBSTACLE_AVOIDANCE_GAIN = 0.6
        self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE = 0.16  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE = 0.12  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR = 0.25
        self.FOV = 200  # degrees
        self.FOV_DISTANCE = 25  # Number of grid cells
        self.FOV_DEADZONE = 80  # degrees
        self.SMALL_FOV = 300  # degrees
        self.SMALL_FOV_DISTANCE = 10  # Number of grid cells

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(2.0, self.check_transform)

        self.pose = None
        self.map = None
        self.path = Path()
        self.alpha = 0
        self.enabled = True
        self.reversed = False
        self.closest_distance = float("inf")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pure_pursuit_sub = self.create_subscription(
            Path, "/pure_pursuit/path", self.update_path, qos
        )

        # test_msg = Path()
        # test_msg.header.frame_id = 'map'
        # self.update_path(test_msg)

    def check_transform(self):
 
        if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
            self.timer = self.create_timer(1.0, self.get_odom)  # Start main loop

        else:
            self.get_logger().warn("odom NOT available!")

    def get_odom(self, msg: Union[Odometry, None]=None):
        try:
            transform_base_to_map = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), Duration(seconds=1))
            trans = transform_base_to_map.transform.translation
            rot = transform_base_to_map.transform.rotation
            self.pose = Pose(
                position=Point(x=trans.x, y=trans.y),
                orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
            )
#            print('pose updated')

        except tf2_ros.TransformException or tf2_ros.LookupException:
            print("didnt work")




    def update_odometry(self, msg: Union[Odometry,None] = None):
        """
        Updates the current pose of the robot.
        """
        # try:
        #     # transform_map_to_odom = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(), Duration(seconds=1))
        #     # transform_odom_to_base = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        #     # transform_base_to_map = self.tf_buffer.transform(transform_odom_to_base, "map")
        #     transform_base_to_map = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), Duration(seconds=1))
        #     trans = transform_base_to_map.transform.translation
        #     rot = transform_base_to_map.transform.rotation

        #     self.pose = Pose(
        #         position=Point(x=trans.x, y=trans.y),
        #         orientation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w),
        #     )
        #     print('pose updated')

        # except tf2_ros.TransformException or tf2_ros.LookupException:
        #     print("didnt work")


    def update_map(self, msg: OccupancyGrid):
        """
        Updates the current map.
        This method is a callback bound to a Subscriber.
        :param msg [OccupancyGrid] The current map information.
        """
        self.map = msg

    def update_path(self, msg: Path):
        self.path = msg
        self.get_logger().info(f"Received path message with {len(self.path.poses)} poses")
        for i, pose in enumerate(self.path.poses):
            self.get_logger().info(f"Waypoint {i}: ({pose.pose.position.x}, {pose.pose.position.y})")
        

    def update_enabled(self, msg: Bool):
        self.enabled = msg.data

    def calculate_steering_adjustment(self) -> float:
        if self.pose is None or self.map is None:
            return 0

        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        yaw = float(np.rad2deg(yaw))

        # Get the grid cell of the robot
        robot_cell = PathPlanner.world_to_grid(self.map, self.pose.position)

        weighted_sum_of_angles = 0
        total_weight = 0
        self.closest_distance = float("inf")

        # Get all wall cells near the robot within the distance
        fov_cells = []
        wall_cells = []
        wall_cell_count = 0
        for dx in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
            for dy in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
                cell = (robot_cell[0] + dx, robot_cell[1] + dy)
                distance = PathPlanner.euclidean_distance(robot_cell, cell)

                # If the cell is out of bounds, ignore it
                if not PathPlanner.is_cell_in_bounds(self.map, cell):
                    continue

                is_wall = not PathPlanner.is_cell_walkable(self.map, cell)   #is_walkable only check for bounds not walls need to check if cell = 100 or >50 or smth
                if is_wall and distance < self.closest_distance:
                    self.closest_distance = distance

                # Calculate the angle of the cell relative to the robot
                angle = float(np.rad2deg(np.arctan2(dy, dx))) - yaw

                # If reversed, add 180 to the angle
                if self.reversed:
                    angle += 180

                # Keep angle in the range of -180 to 180
                if angle < -180:
                    angle += 360
                elif angle > 180:
                    angle -= 360

                # Ignore scans that are outside the field of view
                is_in_fov = (
                    distance <= self.FOV_DISTANCE
                    and angle >= -self.FOV / 2
                    and angle <= self.FOV / 2
                    and not abs(angle) < self.FOV_DEADZONE / 2
                )
                is_in_small_fov = (
                    distance <= self.SMALL_FOV_DISTANCE
                    and angle >= -self.SMALL_FOV / 2
                    and angle <= self.SMALL_FOV / 2
                )
                if not is_in_fov and not is_in_small_fov:
                    continue

                # If in debug mode, add the cell to the field of view
                if self.is_in_debug_mode:
                    fov_cells.append(cell)

                # If cell is not a wall, ignore it
                if not is_wall:
                    continue

                weight = 1 / (distance**2) if distance != 0 else 0

                weighted_sum_of_angles += weight * angle
                total_weight += weight

                wall_cell_count += 1

                if self.is_in_debug_mode:
                    wall_cells.append(cell)

        # If in debug mode, publish the wall cells
        if self.is_in_debug_mode:
            self.fov_cells_pub.publish(PathPlanner.get_grid_cells(self.map, fov_cells))
            self.close_wall_cells_pub.publish(
                PathPlanner.get_grid_cells(self.map, wall_cells)
            )

        if total_weight == 0:
            return 0

        # Calculate the average angle (weighted sum of angles divided by total weight)
        average_angle = weighted_sum_of_angles / total_weight

        # Calculate the steering adjustment based on the average angle
        steering_adjustment = (
            -self.OBSTACLE_AVOIDANCE_GAIN * average_angle / wall_cell_count
        )
        return steering_adjustment

    @staticmethod
    def distance(x0, y0, x1, y1) -> float:
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def get_distance_to_waypoint_index(self, i: int) -> float:
        if self.pose is None or self.path.poses is None:
            return -1

        position = self.pose.position
        waypoint = self.path.poses[i].pose.position
        return PurePursuit.distance(position.x, position.y, waypoint.x, waypoint.y)

    def find_nearest_waypoint_index(self) -> int:
        nearest_waypoint_index = -1
        if self.path.poses is None:
            return nearest_waypoint_index

        closest_distance = float("inf")
        for i in range(len(self.path.poses) - 1):
            distance = self.get_distance_to_waypoint_index(i)
            if distance and distance < closest_distance:
                closest_distance = distance
                nearest_waypoint_index = i
        return nearest_waypoint_index

    def find_lookahead(self, nearest_waypoint_index, lookahead_distance) -> Point:
        if not self.path.poses:
            return Point()

        i = nearest_waypoint_index
        while (
            i < len(self.path.poses)
            and self.get_distance_to_waypoint_index(i) < lookahead_distance
        ):
            i += 1
        return self.path.poses[i - 1].pose.position

    def get_goal(self) -> Point:
        # try:
        if not self.path.poses:
            return Point()

        poses = self.path.poses
        print("in goal function", len(poses))
        return poses[len(poses) - 1].pose.position
        # except IndexError:
        #     return Point()

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        print("send_speed called")
        print(linear_speed,angular_speed)
        twist = Twist(
            linear=Vector3(x=float(linear_speed), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(angular_speed))
        )

        #twist = Twist(linear=Vector3(x=float(linear_speed) y= 0.0, z=0.0), angular=Vector3(x=0,0, y=0.0, z=float(angular_speed)))
        print(twist,"sent")
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        self.send_speed(0, 0)

    def run(self):
        sleep(5)
        while rclpy.ok():
            try: 
                if self.pose is None:
                    rclpy.spin_once(self)
                    print("pose not got")

                # If not enabled, do nothing
                if not self.enabled:
                    rclpy.spin_once(self)

                # If no path, stop
                if self.path is None or not self.path.poses:
                    print(self.path.poses)
                    rclpy.spin_once(self)
                    print("no path or no or self.path.poses, stopping")
                    self.stop()
                
                goal = self.get_goal()

                nearest_waypoint_index = self.find_nearest_waypoint_index()
                lookahead = self.find_lookahead(
                        nearest_waypoint_index, self.LOOKAHEAD_DISTANCE
                    )

                self.lookahead_pub.publish(
                        PointStamped(header=Header(frame_id="map"), point=lookahead)
                    )

                # Calculate alpha (angle between target and current position)
                if self.pose != None:
                    position = self.pose.position
                    orientation = self.pose.orientation
                    roll, pitch, yaw = euler_from_quaternion(
                            [orientation.x, orientation.y, orientation.z, orientation.w]
                        )
                    x = position.x
                    y = position.y
                    dx = lookahead.x - x
                    dy = lookahead.y - y
                    self.alpha = float(np.arctan2(dy, dx) - yaw)
                    if self.alpha > np.pi:
                        self.alpha -= 2 * np.pi
                    elif self.alpha < -np.pi:
                        self.alpha += 2 * np.pi

                    # If the lookahead is behind the robot, follow the path backwards
                    self.reversed = abs(self.alpha) > np.pi / 2

                    # Calculate the lookahead distance and center of curvature
                    lookahead_distance = PurePursuit.distance(x, y, lookahead.x, lookahead.y)
                    radius_of_curvature = float(lookahead_distance / (2 * np.sin(self.alpha)))

                    # Calculate drive speed
                    drive_speed = (-1 if self.reversed else 1) * self.MAX_DRIVE_SPEED

                    # Stop if at goal
                    distance_to_goal = PurePursuit.distance(x, y, goal.x, goal.y)
                    if distance_to_goal < self.DISTANCE_TOLERANCE:
                        rclpy.spin_once(self)
                        self.stop()
                    # Calculate turn speed
                    turn_speed = self.TURN_SPEED_KP * drive_speed / radius_of_curvature
                    # Obstacle avoicance
                    turn_speed += self.calculate_steering_adjustment()

                    # Clamp turn speed
                    turn_speed = max(-self.MAX_TURN_SPEED, min(self.MAX_TURN_SPEED, turn_speed))

                    # Slow down if close to obstacle
                    if self.closest_distance < self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE:
                        drive_speed *= float(
                                np.interp(
                                    self.closest_distance,
                                [   
                                    self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE,
                                    self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE,
                                ],
                                [self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR, 1],
                            )
                        )

                # Send speed
                    self.send_speed(drive_speed, turn_speed)
                    rclpy.spin_once(self)
            except KeyboardInterrupt:
                self.destroy_node()
                rclpy.shutdown()



if __name__ == "__main__":
    rclpy.init()
    pp = PurePursuit()
    pp.run()
