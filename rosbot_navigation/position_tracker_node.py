import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Empty, Bool
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException
import numpy as np
import math
from threading import Lock
from collections import deque

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PositionTracker(Node):
    def __init__(self):
        super().__init__('position_tracker')
        self.get_logger().info("Position Tracker Node Started")

        # Create a callback group that allows execution of callbacks in parallel
        self.callback_group = ReentrantCallbackGroup()

        # Path publishers
        self.explore_pub = self.create_publisher(Path, '/path_explore', 10)
        self.return_pub = self.create_publisher(Path, '/path_return', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        # To directly control the robot if needed
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action client for Nav2
        self.navigate_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose', 
            callback_group=self.callback_group
        )

        # Subscribe to odometry for stuck detection
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        # Odometry tracking for stuck detection
        self.odometry_buffer = deque(maxlen=20)  # Store last 20 odometry readings
        self.stuck_detection_active = False
        self.stuck_timer = None
        self.recovery_active = False

        # Path objects
        self.explore_path = Path()
        self.explore_path.header.frame_id = 'odom'  # Use odom as fallback
        self.explore_path.poses = []  # Explicitly initialize the poses list

        self.return_path = Path()
        self.return_path.header.frame_id = 'odom'  # Use odom as fallback
        self.return_path.poses = []  # Explicitly initialize the poses list
        self.processed_return_path = None

        # State variables
        self.returning = False
        self.path_lock = Lock()
        self.min_distance_between_points = 0.2  # Minimum distance between path points in meters
        self.home_position = None  # Store the starting position
        
        # Find which frame to use (map or odom)
        self.reference_frame = 'odom'  # Default to odom
        self.base_frame = 'base_link'
        self.determine_frames_timer = self.create_timer(1.0, self.determine_frames, callback_group=self.callback_group)
        
        # Track position timer (will be started after frames are determined)
        self.track_timer = None

        # Trigger to return home
        self.home_sub = self.create_subscription(
            Empty, 
            '/trigger_home', 
            self.start_return_home, 
            10,
            callback_group=self.callback_group
        )

        # PID controller parameters for navigation
        self.linear_kp = 0.5
        self.linear_ki = 0.02
        self.linear_kd = 0.1
        self.angular_kp = 1.0
        self.angular_ki = 0.05
        self.angular_kd = 0.2
        
        # PID controller state
        self.linear_error_sum = 0.0
        self.linear_error_prev = 0.0
        self.angular_error_sum = 0.0
        self.angular_error_prev = 0.0
        self.last_pid_time = None

        # Waypoint navigation parameters
        self.waypoint_index = 0
        self.waypoint_reached_distance = 0.2
        self.waypoint_timer = None
        
        # Final approach parameters (more precise for home position)
        self.final_approach_distance = 1.0  # Distance to start final approach mode
        self.final_approach_active = False
        self.home_position_tolerance = 0.05  # 5cm tolerance for home position
        self.goal_check_patience = 10   # Number of iterations to confirm goal reached
        self.goal_reached_counter = 0

        # Error recovery
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        self.recovery_rotation_speed = 0.5
        self.recovery_rotation_time = 5.0  # Seconds to rotate during recovery
        
        # Initial status
        self.update_status("Starting up and determining available frames")
        
        # Debug counter
        self.path_points_logged = 0

        # Added these variables for idle detection
        self.last_position = None
        self.last_significant_movement_time = self.get_clock().now()
        self.movement_threshold = 0.05  # 5cm movement threshold
        self.idle_duration_threshold = 30.0  # 30 seconds of minimal movement to trigger return

        # Create a timer to check if exploration is complete
        self.exploration_check_timer = self.create_timer(
            1.0,  # Check every second
            self.check_exploration_complete,
            callback_group=self.callback_group
        )

    def update_status(self, status_text):
        """Update the status message"""
        status = String()
        status.data = status_text
        self.status_pub.publish(status)
        self.get_logger().info(status_text)

    def determine_frames(self):
        """Determine which frames to use for tracking"""
        # Try map first (preferred)
        try:
            if self.tf_buffer.can_transform('map', self.base_frame, Time().to_msg(), timeout=rclpy.duration.Duration(seconds=0.1)):
                self.reference_frame = 'map'
                self.get_logger().info(f"Using map frame for position tracking")
                self.explore_path.header.frame_id = 'map'
                self.return_path.header.frame_id = 'map'
            # Then try odom
            elif self.tf_buffer.can_transform('odom', self.base_frame, Time().to_msg(), timeout=rclpy.duration.Duration(seconds=0.1)):
                self.reference_frame = 'odom'
                self.get_logger().info(f"Using odom frame for position tracking")
                self.explore_path.header.frame_id = 'odom'
                self.return_path.header.frame_id = 'odom'
            else:
                self.get_logger().warn("No valid transform found, will try again")
                return
                
            # Start tracking once we have a valid frame
            if self.track_timer is None:
                self.track_timer = self.create_timer(0.5, self.track_pose, callback_group=self.callback_group)
                self.get_logger().info(f"Started position tracking using {self.reference_frame} frame")
            
            # Stop this timer
            self.destroy_timer(self.determine_frames_timer)
            
        except Exception as e:
            self.get_logger().error(f"Error determining frames: {e}")

    def distance_between_poses(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def track_pose(self):
        """Record the robot's position during exploration"""
        if self.returning:
            return

        try:
            current_time = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform(self.reference_frame, self.base_frame, Time().to_msg())
            
            pose = PoseStamped()
            pose.header.stamp = current_time.to_msg()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = trans.transform.rotation

            with self.path_lock:
                # Store the first pose as home position
                if not self.explore_path.poses:
                    self.home_position = pose
                    self.get_logger().info(f"Recorded home position: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
                    self.explore_path.poses.append(pose)
                    self.path_points_logged += 1
                    
                else:
                    # Only add a new point if it's far enough from the last one
                    last_pose = self.explore_path.poses[-1]
                    distance = self.distance_between_poses(pose, last_pose)
                    if distance > self.min_distance_between_points:
                        self.explore_path.poses.append(pose)
                        self.path_points_logged += 1
                        if self.path_points_logged % 10 == 0:  # Log every 10 points
                            self.get_logger().info(f"Added path point {self.path_points_logged}, distance from last: {distance:.2f}m")
                
                # Publish the updated explore path
                path_msg = Path()
                path_msg.header.frame_id = self.reference_frame
                path_msg.header.stamp = current_time.to_msg()
                path_msg.poses = self.explore_path.poses
                self.explore_pub.publish(path_msg)

            if not self.returning:
                self.update_status(f"Exploring: {self.path_points_logged} points recorded")

        except TransformException as ex:
            self.get_logger().warn(f'Transform error: {ex}')
        except Exception as e:
            self.get_logger().error(f'Error in track_pose: {e}')

    def odom_callback(self, msg):
        """Store odometry data for stuck detection"""
        if self.stuck_detection_active:
            self.odometry_buffer.append(msg)

    def check_if_stuck(self):
        """Check if the robot is stuck by analyzing odometry data"""
        if len(self.odometry_buffer) < self.odometry_buffer.maxlen:
            return False  # Not enough data yet
            
        # Extract positions from buffer
        positions = [(odom.pose.pose.position.x, odom.pose.pose.position.y) for odom in self.odometry_buffer]
        
        # Calculate the total distance moved over the buffer window
        total_distance = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            total_distance += math.sqrt(dx*dx + dy*dy)
            
        # Check if robot is stuck (total movement is very small)
        time_window = self.odometry_buffer[-1].header.stamp.sec - self.odometry_buffer[0].header.stamp.sec
        if time_window == 0:
            time_window = 0.1  # Avoid division by zero
            
        avg_speed = total_distance / time_window
        
        # If average speed is very low, robot might be stuck
        return avg_speed < 0.05  # Less than 5cm/s

    def start_recovery(self):
        """Start recovery behavior when robot is stuck"""
        if self.recovery_active:
            return
            
        self.recovery_active = True
        self.update_status(f"Robot appears stuck, starting recovery (attempt {self.recovery_attempts + 1}/{self.max_recovery_attempts})")
        
        # Stop current movement
        self.fallback_stop_robot()
        
        # Start recovery rotation
        cmd = Twist()
        cmd.angular.z = self.recovery_rotation_speed
        self.cmd_vel_pub.publish(cmd)
        
        # Create timer to stop rotation after a while
        self.recovery_timer = self.create_timer(
            self.recovery_rotation_time, 
            self.finish_recovery, 
            callback_group=self.callback_group
        )
        
        self.recovery_attempts += 1

    def finish_recovery(self):
        """Finish recovery rotation and resume navigation"""
        # Stop rotation
        self.fallback_stop_robot()
        
        # Clear the recovery timer
        self.destroy_timer(self.recovery_timer)
        
        # Resume navigation with next waypoint
        self.update_status("Recovery complete, resuming navigation")
        self.recovery_active = False
        
        # If we've tried recovery too many times, skip this waypoint
        if self.recovery_attempts >= self.max_recovery_attempts:
            self.waypoint_index += 1
            self.update_status(f"Maximum recovery attempts reached, skipping to waypoint {self.waypoint_index}")
            self.recovery_attempts = 0

    def process_return_path(self, path):
        """Process the return path while ensuring home position is preserved exactly"""
        if len(path.poses) <= 2:
            return path  # No need to process very short paths
            
        processed = Path()
        processed.header = path.header
        
        # Make sure we have thinned enough points for efficient navigation
        # But maintain enough to follow the path
        if len(path.poses) > 50:
            # Keep every nth point to reduce path complexity
            step = max(1, len(path.poses) // 50)
            processed.poses = []
            
            # Always keep first point (home position) and last point
            processed.poses.append(path.poses[0])  # Home position
            
            # Keep intermediate points at regular intervals
            for i in range(step, len(path.poses) - 1, step):
                processed.poses.append(path.poses[i])
                
            # Add final point if it's not already added
            if processed.poses[-1] != path.poses[-1]:
                processed.poses.append(path.poses[-1])
                
            self.get_logger().info(f"Processed path from {len(path.poses)} to {len(processed.poses)} points")
        else:
            processed.poses = path.poses
        
        return processed

    def start_return_home(self, msg=None):
        """Begin returning to the home position"""
        self.get_logger().info("Return home triggered")
        
        if self.returning:
            self.get_logger().info("Already returning to home")
            return

        # Set returning flag
        self.returning = True
        self.update_status("Starting return to home")

        with self.path_lock:
            if not self.explore_path.poses:
                self.get_logger().error("No explored path to follow!")
                self.fallback_stop_robot()
                return
                
            self.get_logger().info(f"Preparing to follow path with {len(self.explore_path.poses)} points")
            
            # Create the return path (reversed explore path)
            self.return_path = Path()
            self.return_path.header.frame_id = self.reference_frame
            self.return_path.header.stamp = self.get_clock().now().to_msg()
            
            # Reverse the path
            self.return_path.poses = list(reversed(self.explore_path.poses))
            
            # Process the return path - thin out points but keep essential ones
            self.processed_return_path = self.process_return_path(self.return_path)
            
            # Make sure the home position is the EXACT first point
            if self.home_position:
                # Add home position as a special final waypoint for precise navigation
                self.processed_return_path.poses.append(self.home_position)
                
            # Publish the return path
            self.return_pub.publish(self.return_path)
            
        # Start stuck detection
        self.stuck_detection_active = True
        
        # Reset PID controller state
        self.reset_pid_state()
        
        # Initialize return navigation
        self.final_approach_active = False
        self.waypoint_index = 0
        self.goal_reached_counter = 0
        self.recovery_attempts = 0
            
        # Navigate home using PID
        self.navigate_home_with_pid()

    def reset_pid_state(self):
        """Reset PID controller state"""
        self.linear_error_sum = 0.0
        self.linear_error_prev = 0.0
        self.angular_error_sum = 0.0
        self.angular_error_prev = 0.0
        self.last_pid_time = self.get_clock().now()

    def navigate_home_with_pid(self):
        """Navigate home using PID control to follow waypoints"""
        if not self.processed_return_path or not self.processed_return_path.poses:
            self.get_logger().error("No processed return path available")
            return
            
        self.update_status("Navigating home with PID control")
        
        # Create waypoint following timer
        self.waypoint_timer = self.create_timer(
            0.1, 
            self.pid_navigate_to_waypoint, 
            callback_group=self.callback_group
        )
        
        # Create stuck detection timer
        self.stuck_timer = self.create_timer(
            2.0, 
            self.check_and_recover_if_stuck, 
            callback_group=self.callback_group
        )

    def check_and_recover_if_stuck(self):
        """Check if robot is stuck and initiate recovery if needed"""
        if self.recovery_active or self.final_approach_active:
            return
            
        if self.check_if_stuck():
            self.start_recovery()

    def pid_navigate_to_waypoint(self):
        """Use PID controller to navigate to waypoints"""
        if self.recovery_active:
            return
            
        # Check if we've reached the end of the path
        if self.waypoint_index >= len(self.processed_return_path.poses) - 1:
            # Final waypoint - use home position for precise navigation
            if not self.final_approach_active:
                self.start_final_approach()
            return
            
        try:
            # Get current time for PID calculations
            current_time = self.get_clock().now()
            dt = (current_time - self.last_pid_time).nanoseconds / 1e9  # Convert to seconds
            if dt == 0:
                dt = 0.1  # Avoid division by zero
            self.last_pid_time = current_time
            
            # Get current pose
            trans = self.tf_buffer.lookup_transform(self.reference_frame, self.base_frame, Time().to_msg())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            
            # Get target waypoint
            target = self.processed_return_path.poses[self.waypoint_index]
            target_x = target.pose.position.x
            target_y = target.pose.position.y
            
            # Calculate distance and angle to target
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx)
            
            q = trans.transform.rotation
            yaw = quaternion_to_yaw(q)
            
            # Calculate angle difference
            angle_diff = angle - yaw
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi
                
            # PID for linear velocity
            linear_error = distance
            self.linear_error_sum += linear_error * dt
            linear_error_derivative = (linear_error - self.linear_error_prev) / dt
            self.linear_error_prev = linear_error
            
            # Anti-windup for linear PID
            self.linear_error_sum = max(-2.0, min(2.0, self.linear_error_sum))
            
            # PID for angular velocity
            angular_error = angle_diff
            self.angular_error_sum += angular_error * dt
            angular_error_derivative = (angular_error - self.angular_error_prev) / dt
            self.angular_error_prev = angular_error
            
            # Anti-windup for angular PID
            self.angular_error_sum = max(-2.0, min(2.0, self.angular_error_sum))
            
            # Create velocity command using PID
            cmd = Twist()
            
            # If close to waypoint, move to next one
            if distance < self.waypoint_reached_distance:
                self.waypoint_index += 1
                self.get_logger().info(f"Reached waypoint {self.waypoint_index}/{len(self.processed_return_path.poses)} (PID)")
                # Reset PID integrals when reaching a waypoint
                self.linear_error_sum = 0.0
                self.angular_error_sum = 0.0
                return
                
            # If angle is significantly off, prioritize rotation
            if abs(angle_diff) > 0.3:
                # Angular PID
                cmd.angular.z = (
                    self.angular_kp * angular_error +
                    self.angular_ki * self.angular_error_sum +
                    self.angular_kd * angular_error_derivative
                )
                # Limit angular velocity
                cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
            else:
                # Linear PID
                cmd.linear.x = (
                    self.linear_kp * linear_error +
                    self.linear_ki * self.linear_error_sum +
                    self.linear_kd * linear_error_derivative
                )
                # Limit linear velocity based on distance
                cmd.linear.x = max(0.0, min(0.3, cmd.linear.x))
                
                # Angular correction (proportional only when moving)
                cmd.angular.z = self.angular_kp * angular_error
                # Limit angular velocity
                cmd.angular.z = max(-0.8, min(0.8, cmd.angular.z))
                
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
        except TransformException as ex:
            self.get_logger().warn(f'Transform error in PID navigation: {ex}')
        except Exception as e:
            self.get_logger().error(f"Error in PID navigation: {e}")

    def start_final_approach(self):
        """Start final approach mode for precise homing"""
        if self.final_approach_active:
            return
            
        self.final_approach_active = True
        self.update_status("Starting final approach to home position")
        
        # Use higher precision parameters for final approach
        self.waypoint_reached_distance = self.home_position_tolerance
        
        # Stop the robot to reset motion
        self.fallback_stop_robot()
        
        # Create timer for final approach
        if self.waypoint_timer:
            self.destroy_timer(self.waypoint_timer)
            
        self.waypoint_timer = self.create_timer(
            0.1, 
            self.final_approach_to_home, 
            callback_group=self.callback_group
        )

    def final_approach_to_home(self):
        """Execute final approach to home position with high precision"""
        try:
            # Get current pose
            trans = self.tf_buffer.lookup_transform(self.reference_frame, self.base_frame, Time().to_msg())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            
            # Get home position
            home_x = self.home_position.pose.position.x
            home_y = self.home_position.pose.position.y
            
            # Calculate distance and angle to home
            dx = home_x - current_x
            dy = home_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx)
            
            # ✅ CORRECT — use the function to compute yaw from quaternion
            q = trans.transform.rotation
            yaw = quaternion_to_yaw(q)
            angle_diff = angle - yaw

            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi
                
            # Check if we've reached home
            if distance < self.home_position_tolerance:
                self.goal_reached_counter += 1
                
                if self.goal_reached_counter >= self.goal_check_patience:
                    self.on_goal_reached()
                    return
                    
                # Stop and wait for confirmation
                self.fallback_stop_robot()
                return
            else:
                self.goal_reached_counter = 0
                
            # Create velocity command for final approach - much slower and more precise
            cmd = Twist()
            
            # If angle is significantly off, correct rotation first
            if abs(angle_diff) > 0.1:
                # Reduced angular velocity for precision
                cmd.angular.z = 0.3 * angle_diff
                # Limit angular velocity
                cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))
            else:
                # Reduced linear velocity for precision
                cmd.linear.x = min(0.1, distance)
                
                # Small angular correction
                cmd.angular.z = 0.5 * angle_diff
                # Limit angular velocity
                cmd.angular.z = max(-0.3, min(0.3, cmd.angular.z))
                
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f"Error in final approach: {e}")

    def on_goal_reached(self):
        """Called when the goal is successfully reached"""
        self.update_status("Successfully reached home position!")
        self.fallback_stop_robot()
        
        # Clean up timers
        if self.waypoint_timer:
            self.destroy_timer(self.waypoint_timer)
        if self.stuck_timer:
            self.destroy_timer(self.stuck_timer)
            
        # Publish goal reached
        goal_msg = Bool()
        goal_msg.data = True
        self.goal_reached_pub.publish(goal_msg)
        
        # Reset state
        self.stuck_detection_active = False
        self.odometry_buffer.clear()
        
        self.get_logger().info("Navigation complete, robot at home position")

    def fallback_stop_robot(self):
        """Stop the robot as a fallback"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        # Publish the stop command
        self.cmd_vel_pub.publish(cmd)

    def check_exploration_complete(self):
    #Check if the robot has been minimally moving for a while (indicating exploration is done)
        if self.returning:
            return
            
        try:
            # Get current position
            trans = self.tf_buffer.lookup_transform(self.reference_frame, self.base_frame, Time().to_msg())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            current_pos = (current_x, current_y)
            
            # Check if we've moved significantly
            if self.last_position:
                dx = current_x - self.last_position[0]
                dy = current_y - self.last_position[1]
                distance_moved = math.sqrt(dx*dx + dy*dy)
                
                # If significant movement, update the timer
                if distance_moved > self.movement_threshold:
                    self.last_significant_movement_time = self.get_clock().now()
            
            # Update last known position
            self.last_position = current_pos
            
            # Check if we've been idle for too long
            current_time = self.get_clock().now()
            idle_duration = (current_time - self.last_significant_movement_time).nanoseconds / 1e9
            
            if idle_duration > self.idle_duration_threshold and len(self.explore_path.poses) > 10:
                self.get_logger().info(f"Robot appears to have completed exploration (idle for {idle_duration:.1f}s)")
                self.start_return_home()
                
        except Exception as e:
            self.get_logger().warn(f"Error checking exploration status: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = PositionTracker()
    
    # Use a MultiThreadedExecutor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Spinning node")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    except Exception as e:
        node.get_logger().error(f"Error during execution: {e}")
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()