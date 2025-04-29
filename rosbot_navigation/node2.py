import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from find_object_2d.msg import ObjectsStamped
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import math
from std_msgs.msg import Empty  
from std_msgs.msg import String 
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan



class HazardMarkerDetector(Node):
    def _init_(self):
        super()._init_('hazard_marker_detector')

        self.create_subscription(ObjectsStamped, '/objectsStamped', self.object_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/hazards', 10)
        self.start_pub = self.create_publisher(Empty, '/trigger_start', 10)
        self.exploration_started = False
        self.status_pub = self.create_publisher(String, '/snc_status',10)
        self.trigger_home = self.create_publisher(Empty,'/trigger_home',10)

        self.laser_data = None
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.detected_ids = set()
        self.published_marker_ids = set()
        self.return_triggered = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # camera intrinsics
        self.fx = 1034.65740703125
        self.fy = 1034.1529541015625
        self.cx = 618.2791137695312
        self.cy = 348.23451259765625


        # hazard marker ID allocation
        self.hazard_marker = {
            0: "Unknown",
            1: "Explosive",
            2: "Flammable Gas",
            3: "Non-Flammable Gas",
            4: "Dangerous When Wet",
            5: "Flammable Solid",
            6: "Spontaneously Combustible",
            7: "Oxidizer",
            8: "Organic Peroxide",
            9: "Inhalation Hazard",
            10: "Poison",
            11: "Radioactive",
            12: "Corrosive",
            13: "Start"   
        }

        self.publish_status("Waiting for hazards") 

    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"Current Status: {status_str}")

    def laser_callback(self, msg):
        self.laser_data = msg

    def object_callback(self, msg):
        data = msg.objects.data
        if len(data) < 4:
            self.get_logger().info("No object detected!!")
            return

        hazard_id = int(data[0])
        label = self.hazard_marker.get(hazard_id, "Unknown")

        if hazard_id == 13 and not self.exploration_started:
            self.get_logger().info("StART MARKER DETECTED! Triggering exploration")
            # Publish to /trigger_start
            self.start_pub.publish(Empty())  
            self.exploration_started = True
            return  # early exit 
        
        if hazard_id == 13:
            return  # start marker ID 

        # skip this if already detected
        if hazard_id in self.detected_ids:
            return

        self.detected_ids.add(hazard_id)
        self.publish_status(f"Hazard Detected: {label}")
        self.get_logger().info(f"New hazard detected: {label} (ID: {hazard_id})  {len(self.detected_ids)}/5")


        if len(self.detected_ids) >= 5 and not self.return_triggered:
            self.return_triggered = True
            self.get_logger().info("Detected 5 unique hazards. Publishing completion signal.")
            self.trigger_home.publish(Empty())
            self.publish_status("hazard_detection_complete")
            return

        
        x_raw, y_raw, z_raw = data[3], data[4], data[5]
        self.get_logger().info(f"Raw 3D: x={x_raw}, y={y_raw}, z={z_raw}")

        pixel_x = 640  # X pixel (center of 1280x720 image)
        pixel_y = 360  # Y pixel (center of image)


        fixed_z = 1.2  # meters - aka height

        X = (pixel_x - self.cx) * fixed_z / self.fx
        Y = (pixel_y - self.cy) * fixed_z / self.fy
        Z = fixed_z

        self.get_logger().info(f"Projected 3D (Fixed-Z): x={X:.2f}, y={Y:.2f}, z={Z:.2f}")

        try:
            point = PointStamped()
            point.header.stamp = msg.header.stamp

            point.header.frame_id = "camera_color_optical_frame" 

            # Patch wrong frame_id if needed
            # if point.header.frame_id == "oak_rgb_camera_optical_frame":
            #     self.get_logger().warn("Patching frame_id from 'oak_rgb_camera_optical_frame' to 'oak_camera_rgb_camera_optical_frame'")
            #     point.header.frame_id = "oak_camera_rgb_camera_optical_frame"

            point.point.x = X
            point.point.y = Y
            point.point.z = Z


            self.get_logger().info(f"Position in camera frame: x={point.point.x:.2f}, y={point.point.y:.2f}, z={point.point.z:.2f}")

            # transform = self.tf_buffer.lookup_transform(
            #     'map',
            #     point.header.frame_id,
            #     rclpy.time.Time(),
            #     timeout=rclpy.duration.Duration(seconds=1.0)
            # )

            transform = self.tf_buffer.lookup_transform(
                'map',               # TO
                'camera_color_optical_frame',        # FROM
                # rclpy.time.Time(),

                point.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )


            point_in_map = do_transform_point(point, transform)

            x = point_in_map.point.x
            y = point_in_map.point.y
            z = point_in_map.point.z

            self.get_logger().info(f"Object in map frame: x={x:.2f}, y={y:.2f}, z={z:.2f} (Label: {label})")

            self.get_logger().info(f"Object in map frame: x={x:.2f}, y={y:.2f} (Label: {label})")

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = msg.header.stamp
            marker.ns = "hazards"
            # marker.id = hazard_id
            marker.id = len(self.detected_ids)

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point_in_map.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            self.get_logger().info(f"Marker published for {label} (ID: {hazard_id})")

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HazardMarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down node cleanly.")
    except Exception as e:
        node.get_logger().error(f"Node errored out: {e}")
    finally:
        node.get_logger().info("Destroying node...")
        node.destroy_node()
        rclpy.shutdown()
        print("HazardMarkerDetector node shutdown complete.")

if __name__ == '_main_':
    main()