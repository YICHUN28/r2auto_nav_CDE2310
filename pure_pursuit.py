import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, Vector3
from std_msgs.msg import Bool
from typing import Union
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.duration import Duration
import math
from time import time, sleep 
from path_planner import PathPlanner
import threading
from rclpy.qos import qos_profile_sensor_data

class Smth(Node):
    def __init__(self):
        super().__init__("yawtrial")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0, self.check_transform)

        self.odomsub = self.create_subscription(Odometry,"/odom",self.get_odom,10)

        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel", 10)

        self.mapsub = self.create_subscription(OccupancyGrid,'/map',self.update_map,10)

        self.pathsub = self.create_subscription(Path, "/pure_pursuit/path", self.update_path,qos_profile_sensor_data)

        self.navpub = self.create_publisher(Bool, "/navigatingyesno", 10)

        self.yaw = 0

        self.map = None
        
        self.path = None

        self.goalreached = False

        self.is_navigating = Bool()
        self.is_navigating.data = False

        self.spin_thread = threading.Thread(target=rclpy.spin,args=(self,))
        self.spin_thread.daemon = True
        self.spin_thread.start()    
    def update_map(self, msg: OccupancyGrid):
        self.map = msg
    
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
                orientation = self.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
                self.yaw = yaw
                # print(self.pose.position)
                # print(self.yaw)


            except tf2_ros.TransformException or tf2_ros.LookupException:
                print("didnt work")
    
    def update_path(self,msg:Path):
        if self.is_navigating.data == True:
            return
        
        self.path = msg
        # print(self.path.poses)
        self.get_logger().info(f"Received path message with {len(self.path.poses)} poses")
        # for i, pose in enumerate(self.path.poses):
        #     self.get_logger().info(f"Waypoint {i}: ({pose.pose.position.x}, {pose.pose.position.y})")
        

    def rotate(self):
        # self.get_odom()
        if self.yaw == 0:
            self.goalreached = False
            return 
        if self.path is None:
            print('no path yet')
            return
        
        
        # if self.alpha == None:
        #     return


        # bestpath = [(24, 47), (24, 48), (24, 49), (24, 50), (24, 51), (25, 52), (24, 53), (23, 54), (23, 55),
        #   (23, 56), (23, 57), (23, 58), (23, 59), (23, 60), (23, 61), (23, 62), (23, 63), (23, 64), (23, 65), (23, 66), 
        #   (23, 67), (23, 68), (23, 69), (23, 70), (23, 71), (23, 72), (24, 73), (25, 74), (26, 75), (27, 76), (28, 77), 
        #   (29, 78), (30, 79), (31, 80), (32, 81), (33, 82), (34, 83), (35, 84), (36, 85)]
        
        
        # [(24, 47), (24, 48), (24, 49), (24, 50), (24, 51), (24, 52), (24, 53), (25,57), (25,61), (25,65), (25,70), (24, 73), (25, 74), (26, 75), (27, 76), (28, 77), 
        #   (29, 78), (30, 79), (31, 80), (32, 81), (33, 82), (34, 83), (35, 84), (36, 85)]

        # goal to turn from current orientation at (24,47) to (24, 51)
        self.is_navigating.data = True
        self.navpub.publish(self.is_navigating)

        for i in range(4,len(self.path.poses)-1,4):

            try:

                # print(self.pose.position)
                # self.send_speed(0.0,0.1)

                # coord = PathPlanner.grid_to_world(self.map,bestpath[i])
                lookaheady = self.path.poses[i].pose.position.y
                lookaheadx = self.path.poses[i].pose.position.x
                print(lookaheadx,lookaheady)
                print(self.path.poses[i])

                gridyourown = PathPlanner.world_to_grid(self.map,self.pose.position)
                print(gridyourown)

                worldtogridoftarget = PathPlanner.world_to_grid(self.map,self.path.poses[i].pose.position)
                # yourowny = self.pose.position.x
                # yourownx = self.pose.position.y
                print(worldtogridoftarget)
                print(self.pose.position)
                y_diff = worldtogridoftarget[1]-gridyourown[1]
                x_diff = worldtogridoftarget[0]-gridyourown[0]

                # print(distance)

                alpha = math.atan2(y_diff,x_diff)

                # alpha = math.pi/3   #self.alpha
                angle = alpha - self.yaw
                angvel = 0.4
                print(self.yaw)
                print(alpha)

                if abs(angle) > 0.1:
                    
                    if angle > 0:

                        # if alpha > 0 and self.yaw < 0:
                        #     angle = math.pi+self.yaw + (math.pi - alpha)
                        #     timeend = time() +angle/angvel
                        #     while time() < timeend:
                        #         self.send_speed(0.0,-0.4)
  
                        # print(time())
                        timeend = time() + angle/angvel
                        # print(timeend)
                    
                        while time() < timeend:
                            self.send_speed(0.0,0.4)
                            # print("rotation sent")
                        
                    else:
                        
                        # if alpha < 0 and self.yaw > 0:
                        #     angle = math.pi-self.yaw + (math.pi + alpha)
                        #     timeend = time() +angle/angvel
                        #     while time() < timeend:
                        #         self.send_speed(0.0,0.4)
                        # print(time())
                        timeend = time() + abs(angle)/angvel
                        # print(timeend)
                    
                        while time() < timeend:
                            self.send_speed(0.0,-0.4)
                            # print("rotation sent")

                while True:
                    distance = math.sqrt((lookaheady-self.pose.position.y)**2 + (lookaheadx-self.pose.position.x)**2)

                    if distance < 0.09:
                        break
                    
                    print("distance to goal is",distance)

                    twist = Twist(
                linear=Vector3(x=float(0.2), y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=float(0))
            )
                    self.cmd_vel_pub.publish(twist)
                    # self.send_speed(0.3,0)
                    # print("sent speed")

                    sleep(0.1)


                self.send_speed(0.0,0.0)
                print("sent stop")
            except Exception as e:
                return
            
        self.is_navigating.data = False
        self.navpub.publish(self.is_navigating)
        print("goal reached")
        self.send_speed(0.0,0.0)
        twist = Twist(
            linear=Vector3(x=float(0), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(0))
        )
        self.cmd_vel_pub.publish(twist)

        
        self.path = None


    


    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # print("send_speed called")
        # print(linear_speed,angular_speed)
        twist = Twist(
            linear=Vector3(x=float(linear_speed), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(angular_speed))
        )
        self.cmd_vel_pub.publish(twist)

    def run(self):
        self.get_odom()
if __name__ == "__main__":
    rclpy.init()
    smth = Smth()
    while rclpy.ok():
        smth.rotate()
        if smth.goalreached == True:
            smth.send_speed(0.0,0.0)
            smth.destroy_node()
            rclpy.shutdown()

            break
