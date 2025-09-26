import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import utm

class GPS(Node):
    def __init__(self):
        super().__init__('GPS_integration')

        # === USER INPUT: Real GPS coordinates ===
        self.origin_lat = 12.9915
        self.origin_lon = 80.2337

        self.target_lat = 12.9916
        self.target_lon = 80.2338

        # === Convert origin and target to UTM (meters) ===
        self.origin_utm_x, self.origin_utm_y, self.zone_number, self.zone_letter = utm.from_latlon(self.origin_lat, self.origin_lon)
        target_utm_x, target_utm_y, _, _ = utm.from_latlon(self.target_lat, self.target_lon)

        # === Compute target (x, y) in local robot frame ===
        self.gps_x = target_utm_x - self.origin_utm_x
        self.gps_y = target_utm_y - self.origin_utm_y

        self.flag = 1

        self.publisher_ = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/bcr_bot/gps_fix', 10)
        self.create_subscription(Odometry, '/bcr_bot/odom', self.time_callback, 10)

        self.get_logger().info(f"Target GPS converted to local frame: x={self.gps_x:.2f}, y={self.gps_y:.2f}")

        #PID constants
        self.kp = 0.8
        self.kd = 0.05
        self.ki = 0.01
        self.kp_ang = 1.0
        self.kd_ang = 0.1
        self.ki_ang = 0.005
        self.linear_prev_error = 0.0
        self.angular_prev_error = 0.0
        self.linear_integral = 0.0
        self.angular_integral = 0.0

    def time_callback(self, msg):
        if self.flag == 0:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # === Convert local x, y to global UTM and back to GPS ===
        curr_utm_x = self.origin_utm_x + x
        curr_utm_y = self.origin_utm_y + y
        latitude, longitude = utm.to_latlon(curr_utm_x, curr_utm_y, self.zone_number, self.zone_letter)

        # === Publish real-time GPS coordinates ===
        gps_msg = NavSatFix()
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = 0.0
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps_link'
        self.gps_pub.publish(gps_msg)

        dx = self.gps_x - x
        dy = self.gps_y - y
        distance = math.hypot(dx, dy)

        if distance < 0.05:
            self.flag = 0
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.publisher_.publish(vel)
            self.get_logger().info('Reached GPS coordinates')
            return
        
        elif distance < 3.0:
            #PID
            angle_to_goal = math.atan2(dy, dx)
            yaw = self.get_yaw(msg.pose.pose.orientation)
            angle_diff = self.normalize_angle(angle_to_goal - yaw)
            vel = Twist()
            # Linear PID
            linear_error = distance
            self.linear_integral += linear_error
            self.linear_integral = max(min(self.linear_integral, 5.0), -5.0) #Prevent linear overshooting
            linear_derivative = linear_error - self.linear_prev_error
            self.linear_prev_error = linear_error
            vel.linear.x = min(
                3.0,
                self.kp * linear_error + self.ki * self.linear_integral + self.kd * linear_derivative
            )

            # Angular PID
            angular_error = angle_to_goal - yaw
            angular_error = self.normalize_angle(angular_error)
            self.angular_integral += angular_error
            self.angular_integral = max(min(self.angular_integral, 2.0), -2.0) #Prevent angular overshooting
            angular_derivative = angular_error - self.angular_prev_error
            self.angular_prev_error = angular_error
            vel.angular.z = (
                self.kp_ang * angular_error
                + self.ki_ang * self.angular_integral
                + self.kd_ang * angular_derivative
            )
            # vel.linear.x =  min(3.0, 0.8*distance) #0.8 is the Propulsion factor Kp (linear)
            # vel.angular.z = 1.0 * angle_diff #1.0 is the Propulsion factor Kp (angular)
            self.publisher_.publish(vel)
            self.get_logger().info('Within distance threshold, starting PID control')
        else:
            angle = math.atan2(dy, dx)
            yaw = self.get_yaw(msg.pose.pose.orientation)
            angle_diff = self.normalize_angle(angle - yaw)

            vel = Twist()
            if abs(angle_diff) > 0.05:
                vel.linear.x = 0.0
                vel.angular.z = 0.5 * (angle_diff / abs(angle_diff))
            else:
                vel.linear.x = 5.0
                vel.angular.z = 0.0

            self.publisher_.publish(vel)

    def get_yaw(self, orientation_q):
        w = orientation_q.w
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y**2 + z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = GPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
