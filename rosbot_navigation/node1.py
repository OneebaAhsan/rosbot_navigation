#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from rclpy.duration import Duration
from std_msgs.msg import String 

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.trigger_sub = self.create_subscription(Empty, '/trigger_start', self.trigger_callback, 10)
        self.teleop_sub = self.create_subscription(Empty, '/trigger_teleop', self.teleop_callback, 10)
        self.home_sub = self.create_subscription(Empty, '/trigger_home', self.home_callback, 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.get_logger().info("NavigationNode (Wall Follower) started \u2014 right-hand rule")

        self.just_turned_right = False
        self.turn_timestamp = self.get_clock().now()
        self.commit_duration = Duration(seconds=1.5)
        self.exploration_complete = False
        self.exploration_started = False
        self.teleop_mode = False

    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"SNC Status: {status_str}")

    def trigger_callback(self, msg):
        self.get_logger().info("'START' marker detected! Triggering exploration...")
        self.exploration_started = True
        self.publish_status("started exploring")

    def teleop_callback(self, msg):
        self.get_logger().warn("Teleop mode triggered! Switching to manual control.")
        self.teleop_mode = True
        self.stop_robot()
        self.publish_status("Teleop command received")

    def home_callback(self, msg):
        self.get_logger().warn("'HOME' command received! Halting exploration.")
        self.exploration_started = False
        self.stop_robot()
        # self.publish_status("Return home")

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.get_logger().warn("\u2714 Robot stopped.")

    def map_callback(self, msg):
        if self.exploration_complete or not self.exploration_started or self.teleop_mode:
            return

        data = msg.data
        total_known = sum(1 for cell in data if cell != -1)
        total_free = sum(1 for cell in data if cell == 0)

        if total_known > 0:
            coverage = total_free / total_known
            self.get_logger().info(f"Map coverage: {coverage:.2f}")

            if coverage > 0.95:
                self.exploration_complete = True
                self.stop_robot()
                self.publish_status("complete")

    def scan_callback(self, msg):
        if self.exploration_complete or not self.exploration_started or self.teleop_mode:
            return

        num_ranges = len(msg.ranges)
        degrees_per_index = 270 / num_ranges

        def angle_to_index(angle):
            return int(angle / degrees_per_index)

        def valid_ranges(range_list):
            return [r for r in range_list if r > 0.0 and r < float('inf')]

        front_indices = list(range(angle_to_index(0), angle_to_index(40))) + list(range(angle_to_index(280), num_ranges))
        front = min(valid_ranges([msg.ranges[i] for i in front_indices]), default=3.0)

        right_indices = range(angle_to_index(220), angle_to_index(270))
        right = min(valid_ranges([msg.ranges[i] for i in right_indices]), default=1.0)

        left_indices = range(angle_to_index(60), angle_to_index(120))
        left = min(valid_ranges([msg.ranges[i] for i in left_indices]), default=1.0)

        twist = Twist()
        now = self.get_clock().now()

        if self.just_turned_right and (now - self.turn_timestamp) < self.commit_duration:
            if right < 0.3:
                twist.linear.x = 0.0
                twist.angular.z = 0.8
                self.get_logger().warn("[Post-Turn] Too close to wall! Emergency turn")
            elif right > 0.4:
                twist.linear.x = 0.10
                twist.angular.z = -0.3
                self.get_logger().warn("[Post-Turn] Realigning with right wall")
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.get_logger().warn("[Post-Turn] Driving straight")
        else:
            self.just_turned_right = False

            if front < 0.3:
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                self.get_logger().warn("Wall ahead \u2192 turning left")

            elif right > 0.8 and front > 1.0:
                twist.linear.x = 0.0
                twist.angular.z = -0.4
                self.get_logger().warn("\u21b1 Turning into right-hand corner")
                self.just_turned_right = True
                self.turn_timestamp = now

            elif right < 0.3:
                twist.linear.x = 0.0
                twist.angular.z = 0.8
                self.get_logger().warn("Too close to wall! Emergency turn")

            elif right > 0.5:
                twist.linear.x = 0.10
                twist.angular.z = -0.3
                self.get_logger().warn("\u2b05 Too far from right wall \u2192 curving right")

            elif right < 0.4:
                twist.linear.x = 0.10
                twist.angular.z = 0.3
                self.get_logger().warn("\u27a1 Too close to right wall \u2192 curving left")

            else:
                twist.linear.x = 0.10
                twist.angular.z = 0.0
                self.get_logger().warn("Driving straight")

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f"[Ranges] Right: {right:.2f} | Front: {front:.2f} | Left: {left:.2f} \u2192 Cmd: Lin {twist.linear.x:.2f}, Ang {twist.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
