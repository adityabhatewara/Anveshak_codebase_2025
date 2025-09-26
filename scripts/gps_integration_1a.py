import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import utm
from sensor_msgs.msg import LaserScan
import numpy as np

DT = 0.1
PREDICT_TIME = 3.0
MAX_SPEED = 1.0
MIN_SPEED = 0.3
MAX_YAWRATE = 1.0
MAX_ACCEL = 0.2
MAX_DYAWRATE = 1.0

# Cost function weights
alpha = 2.0  # heading
beta = 1.5   # obstacle distance
gamma = 0.5   # velocity

class GPS(Node):
    def __init__(self):
        super().__init__('GPS_integration')

        # === USER INPUT: Real GPS coordinates ===
        self.origin_lat = 12.99150
        self.origin_lon = 80.23370

        self.target_lat = 12.99155
        self.target_lon = 80.23375

        # === Convert origin and target to UTM (meters) ===
        self.origin_utm_x, self.origin_utm_y, self.zone_number, self.zone_letter = utm.from_latlon(self.origin_lat, self.origin_lon)
        target_utm_x, target_utm_y, _, _ = utm.from_latlon(self.target_lat, self.target_lon)

        # === Compute target (x, y) in local robot frame ===
        self.gps_x = target_utm_x - self.origin_utm_x
        self.gps_y = target_utm_y - self.origin_utm_y
        self.goal = np.array([self.gps_x, self.gps_y])

        self.flag = 1

        self.publisher_ = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/bcr_bot/gps_fix', 10)
        self.create_subscription(Odometry, '/bcr_bot/odom', self.time_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/bcr_bot/scan', self.dist_from_obstacle_callback, 10)

        self.obstacle_pts = []
        self.state = np.array([0.0, 0.0, 0.0])  # x, y, yaw
        self.v = 0.0
        self.w = 0.0
        self.timer = self.create_timer(DT, self.timer_callback)

        self.get_logger().info(f"Target GPS converted to local frame: x={self.gps_x:.2f}, y={self.gps_y:.2f}")
        self.get_logger().info("DWA controller running with dynamic obstacle avoidance")

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

    def time_callback(self, msg: Odometry):
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
        
        elif distance < .0:
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
            pose = msg.pose.pose
            twist = msg.twist.twist
            self.state[0] = pose.position.x
            self.state[1] = pose.position.y
            yaw = self.get_yaw(pose.orientation)
            self.state[2] = yaw
            self.v = twist.linear.x
            self.w = twist.angular.z

    def dist_from_obstacle_callback(self, msg: LaserScan):
        self.obstacle_pts = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and r < 2.5:  # filter far-away noise
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                self.obstacle_pts.append((x, y))

    def timer_callback(self):
        distance_to_goal = np.hypot(self.goal[0] - self.state[0], self.goal[1] - self.state[1])
        if distance_to_goal < 0.1:
            self.get_logger().info("Goal reached!")
            self.publish_stop()
            rclpy.shutdown()
            return

        u, _ = self.dwa_control(self.state, self.v, self.w)
        cmd = Twist()
        cmd.linear.x = u[0]
        cmd.angular.z = u[1]
        self.publisher_.publish(cmd)

    def dwa_control(self, state, v, w):
        dw = self.calc_dynamic_window(v, w)
        best_cost = float("inf")
        best_u = [0.0, 0.0]

        for vt in np.linspace(dw[0], dw[1], 5):
            for wt in np.linspace(dw[2], dw[3], 5):
                traj = self.calc_trajectory(state.copy(), vt, wt)
                heading = self.calc_heading_cost(traj)
                dist_cost = self.calc_obstacle_cost(traj)
                vel_cost = MAX_SPEED - vt
                total_cost = alpha * heading + beta * dist_cost + gamma * vel_cost

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_u = [vt, wt]
        return best_u, []

    def calc_dynamic_window(self, v, w):
        v_min = max(MIN_SPEED, v - MAX_ACCEL * DT) if v < MIN_SPEED else max(0.0, v - MAX_ACCEL * DT)
        return [
            v_min,
            min(MAX_SPEED, v + MAX_ACCEL * DT),
            max(-MAX_YAWRATE, w - MAX_DYAWRATE * DT),
            min(MAX_YAWRATE, w + MAX_DYAWRATE * DT)
        ]

    def calc_trajectory(self, state, v, w):
        traj = []
        for _ in range(int(PREDICT_TIME / DT)):
            state = self.motion(state, v, w)
            traj.append(state.copy())
        return np.array(traj)

    def motion(self, state, v, w):
        x, y, theta = state
        x += v * np.cos(theta) * DT
        y += v * np.sin(theta) * DT
        theta += w * DT
        return np.array([x, y, theta])

    def calc_heading_cost(self, traj):
        dx = self.goal[0] - traj[-1][0]
        dy = self.goal[1] - traj[-1][1]
        goal_theta = np.arctan2(dy, dx)
        error = goal_theta - traj[-1][2]
        return abs(np.arctan2(np.sin(error), np.cos(error)))

    def calc_obstacle_cost(self, traj):
        if not self.obstacle_pts:
            return 0.0

        min_dist = float('inf')
        for x, y, _ in traj:
            for ox_r, oy_r in self.obstacle_pts:
                yaw = self.state[2]
                # Transform from robot to world frame
                ox_w = ox_r * math.cos(yaw) - oy_r * math.sin(yaw) + self.state[0]
                oy_w = ox_r * math.sin(yaw) + oy_r * math.cos(yaw) + self.state[1]
                dist = np.hypot(ox_w - x, oy_w - y)
                if dist < min_dist:
                    min_dist = dist

        if min_dist < 0.3:
            return float('inf')  # collision can happen, discourage this path as much as possible
        return 2.0 / min_dist  # closer is worse (more the distance, lower the cost)
    
    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)


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
