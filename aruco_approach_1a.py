import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import utm
from ros2_aruco_interfaces.msg import ArucoMarkers


class GPS(Node):
    def __init__(self):
        super().__init__('GPS_integration')

        # origin GPS
        self.origin_lat = 12.9915
        self.origin_lon = 80.2337

        # list of target GPS coordinates
        self.targets = [
            (12.9916, 80.2338),  # Goal 1
            (12.9916, 80.2337),  # Goal 2
            (12.9915, 80.2338),  # Goal 3
        ]
        self.aruco_callback_count=0
        self.id_order=[124, 120, 128]
        self.found_targets={} # Maps ArUco ID to GPS coord
        self.mapping_done=False # Flag: mapping not done yet
        self.current_goal_index = 0
        # ArUco ID storage
        self.aruco_ids = []

        # Convert origin to UTM
        self.origin_utm_x, self.origin_utm_y, self.zone_number, self.zone_letter = utm.from_latlon(self.origin_lat, self.origin_lon)

        # Initialize current target
        self.update_target_coords()

        self.flag = 1

        self.publisher_ = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/bcr_bot/gps_fix', 10)

        self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)

        self.bot_x = None
        self.bot_y = None
        self.bot_yaw = None
        self.aruco_pose = None  # ArUco position (x, y) in world frame

        self.get_logger().info(f"Target GPS converted to local frame: x={self.gps_x:.2f}, y={self.gps_y:.2f}")

    def update_target_coords(self):
        lat, lon = self.targets[self.current_goal_index]
        target_utm_x, target_utm_y, _, _ = utm.from_latlon(lat, lon)
        self.gps_x = target_utm_x - self.origin_utm_x
        self.gps_y = target_utm_y - self.origin_utm_y
        self.get_logger().info(f"New target: x={self.gps_x:.2f}, y={self.gps_y:.2f}")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.bot_x = x
        self.bot_y = y
        self.bot_yaw = self.get_yaw(msg.pose.pose.orientation)

        # Publish simulated GPS
        curr_utm_x = self.origin_utm_x + x
        curr_utm_y = self.origin_utm_y + y
        lat, lon = utm.to_latlon(curr_utm_x, curr_utm_y, self.zone_number, self.zone_letter)

        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = 0.0
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps_link'
        self.gps_pub.publish(gps_msg)

        # Main logic starts here
        dx = self.gps_x - x
        dy = self.gps_y - y
        distance = math.hypot(dx, dy)

        vel = Twist()

        if distance < 5.00:
            self.get_logger().info('GPS unreliable now, switching to PID')
            print(distance)
            self.flag = 0
            vel = Twist()
            angle = math.atan2(dy, dx)
            yaw = self.get_yaw(msg.pose.pose.orientation)
            angle_diff = self.normalize_angle(angle - yaw)
            vel.linear.x = min(3.0, 0.8 * distance)  # 0.8 is the Propulsion factor Kp (linear)
            vel.angular.z = 1.0 * angle_diff  # 1.0 is the Propulsion factor Kp (angular)
            self.publisher_.publish(vel)

            if distance < 1.00:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                self.publisher_.publish(vel)

                if not self.mapping_done:
                    self.get_logger().info(f"Scanned location {self.current_goal_index + 1}. Checking for markers...")
                    self.current_goal_index += 1

                    if self.current_goal_index < len(self.targets):
                        self.update_target_coords()
                        self.flag = 1
                    else:
                        # All locations visited; check if all required IDs are found
                        #if all(i in self.found_targets for i in self.id_order):
                            self.get_logger().info("All required markers found. Reordering targets by ID.")
                            self.targets = [self.found_targets[i] for i in self.id_order]
                            self.current_goal_index = 0
                            self.mapping_done = True
                            self.update_target_coords()
                            self.flag = 1
                        # else:
                        #     self.get_logger().warn("Not all markers found. Shutting down.")
                        #     print("Mapped markers:", self.found_targets)
                        #     rclpy.shutdown()
                else:
                    # PHASE 2: Revisiting based on ArUco ID order
                    self.get_logger().info(f"Visited ID-based location {self.current_goal_index + 1}")
                    self.current_goal_index += 1

                    if self.current_goal_index < len(self.targets):
                        self.update_target_coords()
                        self.flag = 1
                    else:
                        self.get_logger().info("Mission complete. All markers revisited in ID order.")
                        rclpy.shutdown()
                return None


        # GPS navigation phase
        if self.flag == 1:
            angle_to_target = math.atan2(dy, dx)
            yaw_diff = self.normalize_angle(angle_to_target - self.bot_yaw)

            if abs(yaw_diff) > 0.05:
                vel.linear.x = 0.0
                vel.angular.z = 0.5 * (yaw_diff / abs(yaw_diff))
            else:
                vel.linear.x = min(3.0, 0.8 * distance)
                vel.angular.z = 0.0

            self.publisher_.publish(vel)

    def get_yaw(self, orientation_q):
        w, x, y, z = orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def aruco_callback(self, msg):
        if self.mapping_done:
            return

        if msg.marker_ids:
            for marker_id in msg.marker_ids:
                if marker_id not in self.found_targets:
                    lat, lon = self.targets[self.current_goal_index]
                    self.found_targets[marker_id] = (lat, lon)
                    self.get_logger().info(f"Mapped ArUco ID {marker_id} to current goal ({lat:.6f}, {lon:.6f})")

            print(f"Detected ArUco IDs: {msg.marker_ids}")




def main(args=None):
    rclpy.init(args=args)
    node = GPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()
