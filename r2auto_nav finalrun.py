# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf_transformations import euler_from_quaternion
import rclpy.time
import numpy as np
import math
import cmath
import time
import random
from std_msgs.msg import Float32, Bool

# constants
rotatechange = 0.5
speedchange = 0.2
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.34
front_angle = 12
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
temp_check_interval = 5.0  # seconds between temperature mode checks
temp_alignment_interval = 3.0  # seconds between alignment scans
temp_threshold = 27.0  # temperature threshold to trigger temp mode
temp_mode_timeout = 20.0  # timeout to exit temp mode if no progress is made


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.position = 0
        self.positions = []
        self.temp = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ismovingtemp = False
        self.positionxy = None
        self.prox = Bool()
        self.prox.data = False
        
        # New variables for improved temperature sensing
        self.last_temp_check_time = time.time()
        self.last_alignment_time = time.time()
        self.temp_mode_start_time = 0
        self.initial_position = None
        self.max_temp = 0
        self.max_temp_angle = 0
        self.is_aligning = False
        self.alignment_angles = [-10, -5, 0, 5, 10]  # Angles to check during alignment
        self.current_alignment_index = 0
        self.alignment_temps = []
        self.alignment_start_yaw = 0

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.tempsub = self.create_subscription(Float32,'/temp_pub',self.temp_update,10)
        self.proxpub = self.create_publisher(Bool, "/proxcheck", 10)
        self.found = Bool()
        self.found.data = False
    def temp_update(self,msg):
        self.temp = msg.data
        print(f"Current temperature: {self.temp}")

        # If not already in temp mode and temp exceeds threshold, enter temp mode
        if not self.ismovingtemp and self.temp > temp_threshold:
            self.ismovingtemp = True
            self.temp_mode_start_time = time.time()
            self.initial_position = self.positionxy
            self.get_logger().info('Entering temperature sensing mode')
            self.max_temp = self.temp
            self.max_temp_angle = self.yaw
    
    def check_temp_mode_status(self):
        """Periodically check if we should still be in temperature sensing mode"""
        current_time = time.time()
        
        # Only check if in temp mode and enough time has passed since last check
        if self.ismovingtemp and (current_time - self.last_temp_check_time) > temp_check_interval:
            self.last_temp_check_time = current_time
            
            # Check if temperature is still above threshold
            if self.temp < temp_threshold - 0.5:  # Hysteresis to prevent oscillation
                self.get_logger().info('Temperature dropped below threshold, exiting temp mode')
                self.ismovingtemp = False
                return
            
            # Check if we've timed out (stuck in temp mode too long)
            if (current_time - self.temp_mode_start_time) > temp_mode_timeout:
                # Check if we've made significant progress toward heat source
                if self.max_temp < 28.0:  # If we haven't found a very hot source
                    self.get_logger().info('Timeout in temp mode without finding hot source, exiting temp mode')
                    self.ismovingtemp = False
                    return
    
    def perform_temp_alignment(self):
        """Scan for the hottest direction and align with it"""
        if not self.ismovingtemp:
            return
        
        current_time = time.time()
        if (current_time - self.last_alignment_time) > temp_alignment_interval:
            # Time to do alignment scan
            if not self.is_aligning:
                # Start alignment process
                self.is_aligning = True
                self.alignment_temps = []
                self.current_alignment_index = 0
                self.stopbot()
                
                # Save starting yaw
                self.alignment_start_yaw = self.yaw
                
                # Start the alignment process
                self.get_logger().info('Starting temperature alignment scan')
                self.rotate_for_alignment()
        
    def rotate_for_alignment(self):
        """Rotate to the next angle in the alignment scan using rotatebot"""
        if self.current_alignment_index >= len(self.alignment_angles):
            # We've completed all scans, find the best angle
            self.complete_alignment_scan()
            return
        
        # Get the next angle to check
        angle_to_check = self.alignment_angles[self.current_alignment_index]
        self.get_logger().info(f'Scanning at angle offset: {angle_to_check} degrees')
        
        # Calculate the relative angle to rotate
        if self.current_alignment_index == 0:
            # First rotation is relative to starting position
            relative_angle = angle_to_check
        else:
            # Subsequent rotations are relative to previous position
            relative_angle = angle_to_check - self.alignment_angles[self.current_alignment_index - 1]
        
        # Use rotatebot to move by the relative angle
        self.rotatebot(relative_angle)
        
        # After rotation completes, record the temperature
        self.alignment_temps.append(self.temp)
        self.get_logger().info(f'Temperature at {angle_to_check} degrees: {self.temp}°C')
        
        # Move to next angle
        self.current_alignment_index += 1
        
        # Wait a moment to stabilize before next measurement
        time.sleep(0.5)
        
        # Continue to next angle
        self.rotate_for_alignment()
    
    def complete_alignment_scan(self):
        """Process alignment scan results and rotate to best angle"""
        self.is_aligning = False
        self.last_alignment_time = time.time()
        
        # Find the angle with the highest temperature
        max_temp_index = np.argmax(self.alignment_temps)
        best_angle = self.alignment_angles[max_temp_index]
        max_temp = self.alignment_temps[max_temp_index]
        
        self.get_logger().info(f'Best direction is {best_angle} degrees with temp {max_temp}°C')
        
        # Update overall max temp if this is higher
        if max_temp > self.max_temp:
            self.max_temp = max_temp
            self.max_temp_angle = self.alignment_start_yaw + math.radians(best_angle)
        
        # Calculate angle to rotate back to the best position
        # We need to go from current position back to the start, then to the best angle
        self.rotatebot(best_angle - self.alignment_angles[self.current_alignment_index - 1])
        
        # Now continue moving toward heat source
        twist = Twist()
        twist.linear.x = 0.1  # Move more slowly toward heat source for better control
        self.publisher_.publish(twist)
    
    def movetotemp(self):
        angvel = 0
        """Move toward temperature source with improved alignment"""
        twist = Twist()
        
        # Regular checks to see if we should exit temp mode
        self.check_temp_mode_status()
        
        # If already in alignment process, skip the rest
        if self.is_aligning:
            return
        
        # Perform periodic alignment scans if not currently aligning
        current_time = time.time()
        if (current_time - self.last_alignment_time) > temp_alignment_interval:
            self.perform_temp_alignment()
            return
        
        print("Moving toward heat source")
        
        # Check if we've reached a very hot temperature
        if self.temp > 30.5:
            if not self.positions:
                self.found.data = True
                self.proxpub.publish(self.found)
                print(self.found)
                self.stopbot()
                angvel = 0.3
                timeend = time.time() + math.pi/angvel

                while time.time() < timeend:
                    twist = Twist()
                    twist.angular.z = angvel
                    self.publisher_.publish(twist)
                self.stopbot()
                self.ismovingtemp = False
                self.positions.append(self.positionxy)

                print(f"Heat source found at position: {self.positions}")
                print("Waiting 10 seconds")
                time.sleep(15)
                time.sleep(1)
                twist.linear.x = speedchange
                self.publisher_.publish(twist)
                
                time.sleep(3)
                self.stopbot()

                self.found.data = False
                self.proxpub.publish(self.found)
                return
            elif math.sqrt((self.positions[0][1]-self.positionxy[1])**2 + (self.positions[0][0] - self.positionxy[0])**2) < 1.2:
                self.found.data = False
                self.proxpub.publish(self.found)
                print("Already found a heat source near this position")
                self.ismovingtemp = False   
                angvel = 0.3
                timeend = time.time() + math.pi/angvel

                while time.time() < timeend:
                    twist = Twist()
                    twist.angular.z = angvel
                    self.publisher_.publish(twist)
                self.stopbot()
                return

        # Check for obstacles
        if self.laser_range.size != 0:
            front_obstacle = np.any(self.laser_range[front_angles] < float(stop_distance))
            if front_obstacle:
                self.stopbot()
                # Define left and right angle ranges
                left_angles = range(0, int(1/6*len(self.laser_range)))
                right_angles = range(int(5/6*len(self.laser_range)), len(self.laser_range))
                
                # Check if obstacle is on the left side
                left_obstacle = np.any(self.laser_range[left_angles] < float(0.25))
                
                # Check if obstacle is on the right side
                right_obstacle = np.any(self.laser_range[right_angles] < float(0.25))
                
                # Determine turn direction based on obstacle position
                if left_obstacle and not right_obstacle:
                    turn_angle = -5
                    self.get_logger().info('Obstacle on left, turning right')
                    self.rotatebot(turn_angle)
                elif right_obstacle and not left_obstacle:
                    turn_angle = 5
                    self.get_logger().info('Obstacle on right, turning left')
                    self.rotatebot(turn_angle)
                else:
                    # If obstacle is directly in front but not on sides, try to find a way around
                    self.rotatebot(10)  # Try turning a bit to find a path
                    return
        
        # Continue moving toward heat source
        self.get_logger().info('Moving toward heat source')
        twist = Twist()
        twist.linear.x = 0.1  # Slow speed for better control
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.position = msg.pose.pose.position
        x = self.position.x
        y = self.position.y
        self.positionxy = (x,y)

        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        if self.ismovingtemp == False:
            self.get_logger().info('In pick_direction')
            if self.laser_range.size != 0:
                # Define left and right angle ranges
                # Assuming laser_range[0] is straight ahead, and angles increase clockwise
                # You may need to adjust these ranges based on your LIDAR configuration
                left_angles = range(0, int(1/6*len(self.laser_range)))  # 0-90 degrees (left side)
                right_angles = range(int(5/6*len(self.laser_range)), len(self.laser_range))  # 270-360 degrees (right side)
                
                # Check if obstacle is on the left side
                left_obstacle = np.any(self.laser_range[left_angles] < float(stop_distance))
                
                # Check if obstacle is on the right side
                right_obstacle = np.any(self.laser_range[right_angles] < float(stop_distance))
                
                # Determine turn direction based on obstacle position
                if left_obstacle and not right_obstacle:
                    # Obstacle on left, turn right (positive angle)
                    turn_angle = -random.randint(10,30)  # Turn right 90 degrees
                    self.get_logger().info('Obstacle on left, turning right')
                elif right_obstacle and not left_obstacle:
                    # Obstacle on right, turn left (negative angle)
                    turn_angle = random.randint(10,30)  # Turn left 90 degrees
                    self.get_logger().info('Obstacle on right, turning left')
                elif left_obstacle and right_obstacle:
                    # Obstacles on both sides, find the clearest direction
                    # Compare average distances on left and right to decide
                    left_avg = np.nanmean(self.laser_range[left_angles])
                    right_avg = np.nanmean(self.laser_range[right_angles])
                    turn_angle = 180
                    
                    if left_avg > right_avg:
                        # Left has more space
                        turn_angle = 45  # Turn left 45 degrees
                        self.get_logger().info('Obstacles on both sides, more space on left')
                        twist = Twist()
                        twist.linear.x = -0.05
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)
                        time.sleep(1.5)
                        self.stopbot()

                    else:
                        # Right has more space
                        turn_angle = -45  # Turn right 45 degrees
                        self.get_logger().info('Obstacles on both sides, more space on right')
                        twist = Twist()
                        twist.linear.x = -0.05
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)
                        time.sleep(1)
                        self.stopbot()
                else:
                    # No obstacles on sides, continue forward or use original logic
                    # You could keep your original direction finding logic here
                    # For now, let's just go forward
                    turn_angle = 0
                    self.get_logger().info('No side obstacles, continuing forward')
                
                # Rotate to the determined direction
                self.rotatebot(turn_angle)
            # else:
            #     self.get_logger().info('No laser data!')
            #     return

            # start moving
            self.get_logger().info('Start moving')
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        # Start moving initially
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        print(twist)

        while rclpy.ok():
            # Check temp mode status if we're in temp mode
            if self.ismovingtemp:
                self.check_temp_mode_status()
            
            # Check if we should enter temp mode
            if self.temp > temp_threshold and not self.ismovingtemp:
                self.stopbot()
                print("Heat detection mode activated")
                self.ismovingtemp = True
                self.temp_mode_start_time = time.time()
                self.initial_position = self.positionxy
                self.max_temp = self.temp
                self.max_temp_angle = self.yaw
                self.movetotemp()

            elif self.laser_range.size != 0 and not self.ismovingtemp:
                # Check for obstacles directly in front
                front_obstacle = np.any(self.laser_range[front_angles] < float(stop_distance))
                # If obstacle directly in front, use our new pick_direction logic
                if front_obstacle:
                    self.stopbot()
                    self.pick_direction()
                else:
                    twist = Twist()
                    twist.linear.x = speedchange
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)

            elif self.ismovingtemp:
                self.movetotemp()   
        
            # Allow the callback functions to run
            rclpy.spin_once(self)
                
def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()