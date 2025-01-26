import csv
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

class WaypointSkipper(Node):
    def __init__(self):
        super().__init__('waypoint_skipper')

        # Parameters
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('waypoint_skip_tolerance', 1.0)
        self.declare_parameter('skip_scan_count', 5)
        self.waypoints_csv = self.get_parameter('waypoints_csv').value
        self.waypoint_skip_tolerance = self.get_parameter('waypoint_skip_tolerance').value
        self.skip_scan_count = self.get_parameter('skip_scan_count').value

        # Subscriber
        self.current_pose = None
        self.current_waypoint_id = None
        self.skip_scan_counter = 0
        self.create_subscription(PoseWithCovarianceStamped, 'current_pose', self.current_pose_callback, 10)
        self.create_subscription(Int32, 'next_waypoint_id', self.waypoint_id_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Publisher
        self.skip_flag_publisher = self.create_publisher(Bool, 'skip_flag', 10)
        self.marker_array_publisher = self.create_publisher(MarkerArray, 'waypoint/skip_tolerance', 10)

        # Load Waypoint CSV file
        self.waypoints_data = self.load_waypoints_from_csv(self.waypoints_csv)
        
        # Check if waypoints data is loaded correctly
        if not self.waypoints_data:
            self.get_logger().error("No waypoints loaded. Please check the CSV file.")
            return

        # Variables to manage markers
        self.last_marker_ids = []

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def waypoint_id_callback(self, msg):
        self.current_waypoint_id = msg.data
        self.publish_circle_marker()

    def scan_callback(self, msg):
        if self.current_pose and self.current_waypoint_id is not None:
            current_waypoint = self.waypoints_data[self.current_waypoint_id]["pose"].pose
            distance_to_waypoint = self.calculate_distance(self.current_pose, current_waypoint)
            
            if distance_to_waypoint <= self.waypoint_skip_tolerance:
                self.skip_scan_counter = 0
                for range_value in msg.ranges:
                    if range_value <= self.waypoint_skip_tolerance:
                        self.skip_scan_counter += 1
                        if self.skip_scan_counter >= self.skip_scan_count:
                            self.skip_flag_publisher.publish(Bool(data=True))
                            return

        self.skip_flag_publisher.publish(Bool(data=False))

    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx * dx + dy * dy)

    def load_waypoints_from_csv(self, waypoints_csv):
        waypoints_data = []
        try:
            with open(waypoints_csv, mode='r') as file:
                reader = csv.reader(file)
                header = next(reader)

                for row in reader:
                    if len(row) < 10:  # Ensure the row has enough columns
                        self.get_logger().error(f"Row in CSV file is too short: {row}")
                        continue
                    pose_stamped_msg = PoseStamped()
                    pose_stamped_msg.header.frame_id = 'map'
                    pose_stamped_msg.pose.position.x = float(row[1])
                    pose_stamped_msg.pose.position.y = float(row[2])
                    pose_stamped_msg.pose.position.z = float(row[3])
                    pose_stamped_msg.pose.orientation.x = float(row[4])
                    pose_stamped_msg.pose.orientation.y = float(row[5])
                    pose_stamped_msg.pose.orientation.z = float(row[6])
                    pose_stamped_msg.pose.orientation.w = float(row[7])

                    waypoint_data = {
                        "pose": pose_stamped_msg,
                        "skip_flag": int(row[8])
                    }

                    waypoints_data.append(waypoint_data)
            
            self.get_logger().info(f"Loaded {len(waypoints_data)} waypoints from {waypoints_csv}.")
        except FileNotFoundError:
            self.get_logger().error(f"File {waypoints_csv} not found.")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
        
        return waypoints_data

    def publish_circle_marker(self):
        marker_array = MarkerArray()
        current_waypoint_skip_flag = self.waypoints_data[self.current_waypoint_id]["skip_flag"]

        # Delete all previous markers
        for marker_id in self.last_marker_ids:
            delete_marker = Marker()
            delete_marker.header.frame_id = 'map'
            delete_marker.action = Marker.DELETE
            delete_marker.id = marker_id
            delete_marker.ns = "waypoint_skipper"
            marker_array.markers.append(delete_marker)

        # Clear the list of previous marker IDs
        self.last_marker_ids.clear()

        # Create and publish a new marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoint_skipper"
        marker.id = self.current_waypoint_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = self.waypoints_data[self.current_waypoint_id]["pose"].pose
        marker.scale.x = self.waypoint_skip_tolerance * 2.0
        marker.scale.y = self.waypoint_skip_tolerance * 2.0
        marker.scale.z = 0.1
        marker.color.a = 0.7
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        if current_waypoint_skip_flag == 0:
            marker.color.r = 1.0
            marker.color.g = 0.0

        marker_array.markers.append(marker)

        # Save the marker ID to remove it later
        self.last_marker_ids.append(marker.id)

        self.marker_array_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    node = WaypointSkipper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
