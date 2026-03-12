
import sys
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import dubins
import utm
import time
# from traversal2.msg import WheelRpm
from geometry_msgs.msg import Twist
from path_generator import find_shortest_path_to_point




class PurePursuit(Node):
    def __init__(self,goal):
        super().__init__("pure_pursuit_controller")

        # --- ROS Parameters --- #
        self.declare_parameter('goal_lat', goal[0])
        self.declare_parameter('goal_lon', goal[1])
        self.declare_parameter('lookahead_distance', 0.7)
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('wheel_radius_m', 0.10) # 10cm radius
        self.declare_parameter('track_width_m', 0.35) # Distance between left and right wheels
        self.declare_parameter('goal_tolerance_m', 0.5)
        self.declare_parameter('control_frequency', 10.0) # Hz

        # --- State Machine ---
        self.state = 'WAITING_FOR_POSE'
        self.get_logger().info(f"Node initialized. Current state: {self.state}")

        # --- Class Attributes ---
        self.pose_odom = None          # Stores (x, y, theta) from odometry
        self.current_lat_lon = None    # Stores (lat, lon) from GPS
        self.path = []
        
        # Get parameters
        self.goal_gps = [
            self.get_parameter('goal_lat').get_parameter_value().double_value,
            self.get_parameter('goal_lon').get_parameter_value().double_value
        ]
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius_m').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width_m').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance_m').get_parameter_value().double_value

        # --- QoS Profiles ---
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # --- Publishers ---
        self.wheelrpm_pub = self.create_publisher(Twist, '/motion', qos_reliable)
        
        # --- Subscribers ---
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_reliable)
        self.create_subscription(Imu, '/zed2i/zed_node/imu/data', self.yaw_callback, qos_best_effort)
        self.create_subscription(NavSatFix, '/gps_coordinates', self.gps_callback, qos_best_effort)

        # --- Control Loop Timer ---
        timer_period = 1.0 / self.get_parameter('control_frequency').get_parameter_value().double_value
        self.control_timer = self.create_timer(timer_period, self.pp_callback)
    
    # --- SUBSCRIBER CALLBACKS --- #
    def odom_callback(self, msg: Odometry):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        orient_w = msg.pose.pose.orientation.w
        orient_x = msg.pose.pose.orientation.x
        orient_y = msg.pose.pose.orientation.y
        orient_z = msg.pose.pose.orientation.z
        
        siny_cosp = 2.0 * (orient_w * orient_z + orient_x * orient_y)
        cosy_cosp = 1.0 - 2.0 * (orient_y**2 + orient_z**2)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.pose_odom = (pos_x, pos_y, theta)

        if self.state == 'WAITING_FOR_POSE':
            self.state = 'GENERATING_PATH'
            self.get_logger().info("Initial odometry received. State changed to GENERATING_PATH.")

    def gps_callback(self, msg: NavSatFix):
        if msg.latitude and msg.longitude:
            self.current_lat_lon = (msg.latitude, msg.longitude)

    def yaw_callback(self,data): ## incase we want to use ZED's odometry ##
        current_x = data.orientation.x
        current_y = data.orientation.y
        current_z = data.orientation.z
        current_w = data.orientation.w
        current_tuple=(current_x, current_y, current_z, current_w)
        current_pitch, current_roll, self.z_angle_in_rad = self.quaternion_to_euler(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)

    

    def get_path(self, rho=1.0, step_size=0.1):
        """Generates a dynamic Dubins path from the current pose to the GPS goal."""
        try:
            print("started")
            # current_utm_x, current_utm_y, _, _ = utm.from_latlon(self.current_lat_lon[0], self.current_lat_lon[1])
            current_utm_x, current_utm_y, _, _ = utm.from_latlon(12.991509823016, 80.23361965386)
            target_utm_x, target_utm_y, _, _ = utm.from_latlon(12.9915090713538, 80.233621802036)
            print("1")
            utm_dx = target_utm_x - current_utm_x
            utm_dy = target_utm_y - current_utm_y

            current_odom_x, current_odom_y, _ = self.pose_odom
            goal_xy_in_odom = (current_odom_x + utm_dx, current_odom_y + utm_dy)

            # path_sparse, _ = self.PathMakerFunction(self.pose_odom, goal_xy_in_odom, rho, step_size)
            path_x, path_y, path_yaw, mode, _ = find_shortest_path_to_point(self.pose_odom[0], self.pose_odon[1], goal_xy_in_odom[0],goal_xy_in_odom[1], curvature = 1.0)
            path_sparse = np.column_stack((path_x, path_y))
            print("2")
            if path_sparse is None:
                self.get_logger().warn("PathMakerFunction returned no path.")
                return False

            # self.path = self.densify_path(path_sparse.tolist(), max_segment_length=0.2)
            self.path = path_sparse

            print("ended")
            return True
        except Exception as e:
            self.get_logger().error(f"Error aagaya🥲 in get_path(): {e}")
            return False


    # def densify_path(self, path, max_segment_length=1.0):
    #     """Inserts points into a path so segments are no longer than max_segment_length."""
    #     new_path = []
    #     if not path:
    #         return []
        
    #     for (x1, y1), (x2, y2) in zip(path, path[1:]):
    #         new_path.append((x1, y1))
    #         dx, dy = x2 - x1, y2 - y1
    #         dist = math.hypot(dx, dy)
    #         if dist > max_segment_length:
    #             num_steps = int(math.ceil(dist / max_segment_length))
    #             for step in range(1, num_steps):
    #                 t = step / num_steps
    #                 xi = x1 + t * dx
    #                 yi = y1 + t * dy
    #                 new_path.append((xi, yi))
                    
    #     new_path.append(path[-1])
    #     return new_path
    
    def isGoalReached(self, tolerance=0.5) -> bool:
        """
        Checks if the rover is within a 'tolerance' distance of the final goal.
        
        Args:
            tolerance (float): The maximum allowed distance (in meters) from the goal.
        
        Returns:
            bool: True if the goal is reached, False otherwise.
        """
        if not self.path:
            # If there is no path, we can't have reached the goal.
            return False

        # Get current and goal positions
        current_x, current_y, _ = self.pose_odom
        goal_x, goal_y = self.path[-1] # The last point in the path is the goal

        distance_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)
        
        if distance_to_goal <= tolerance:
            self.get_logger().info(f"Goal reached! Distance: {distance_to_goal:.2f}m")
            return True
        
        return False


    # --- CONTROL & LOGIC FUNCTIONS --- #
    def pp_callback(self):
        """Main control loop, triggered by a timer."""
        if self.state == 'WAITING_FOR_POSE' or self.pose_odom is None:
            self.get_logger().info("State -> Waiting for pose.")
            return

        if self.state == 'GENERATING_PATH':
            if self.get_path():
                self.get_logger().info(f"Path generated with {len(self.path)} points. State -> FOLLOWING_PATH.")
                self.state = 'FOLLOWING_PATH'
            else:
                self.get_logger().error("Path generation fail hogaya. retry ho raha hai")
            return

        if self.state == 'FOLLOWING_PATH':
            if self.isGoalReached(self.goal_tolerance):
                self.get_logger().info("pauch gaya.")
                self.state = 'GOAL_REACHED'
                self.stop_rover()
            else:
                linear_velocity, angular_velocity = self.pure_pursuit_control()
                left_rpm, right_rpm = self.vel_to_wheel_rpms(linear_velocity, angular_velocity)
                # self.giveRoverPWM(left_rpm, right_rpm)
                msg = Twist()
                msg.linear.x = linear_velocity
                msg.angular.z= angular_velocity
                self.wheelrpm_pub.publish(msg)

        
        if self.state == 'GOAL_REACHED':
            self.get_logger().info("Execution complete. Stopping rover.", once=True)
            self.stop_rover()


    def pure_pursuit_control(self):
        x, y, theta = self.pose_odom

        # 1. Find the closest point on the path to the rover
        _, closest_index = self.get_closest_path_point((x, y))

        # 2. Find the lookahead point on the path
        goal_point = self.find_lookahead_point((x, y), self.lookahead_distance, closest_index)
        
        if goal_point is None: # If no lookahead point is found, maybe we're at the end
            return 0.0, 0.0

        goal_x, goal_y = goal_point

        # 3. Transform goal point to the rover's coordinate frame
        dx = goal_x - x
        dy = goal_y - y
        local_goal_x = math.cos(theta) * dx + math.sin(theta) * dy
        local_goal_y = -math.sin(theta) * dx + math.cos(theta) * dy

        # 4. Calculate the curvature of the arc to the goal point
        alpha = math.atan2(local_goal_y, local_goal_x)
        curvature = (2 * math.sin(alpha)) / self.lookahead_distance

        # 5. Set linear velocity and calculate angular velocity
        linear_velocity = 15.0  # Constant forward speed
        angular_velocity = linear_velocity * curvature
        
        return linear_velocity, angular_velocity
    
    def get_closest_path_point(self, rover_xy):
        """Finds the closest point on the path to the rover."""
        min_dist = float('inf')
        closest_index = 0
        rover_x, rover_y = rover_xy
        for i, (px, py) in enumerate(self.path):
            dist = math.hypot(px - rover_x, py - rover_y)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        return self.path[closest_index], closest_index
    
    def find_lookahead_point(self, rover_xy, lookahead_dist, start_index):
        """Finds a point on the path that is `lookahead_dist` away from the rover."""
        rover_x, rover_y = rover_xy
        for i in range(start_index, len(self.path)):
            px, py = self.path[i]
            dist = math.hypot(px - rover_x, py - rover_y)
            if dist >= lookahead_dist:
                return self.path[i]
        return self.path[-1] # If no point is far enough, return the last point

    
    # def giveRoverPWM(self, left_wheel_rpms, right_wheel_rpms):
    #     """
    #     Publishes the calculated wheel commands as a generic data array.
    #     """
    #     # Create an instance of the Int32MultiArray message
    #     motion_msg = Int32MultiArray()
        
    #     # Create a Python list with the motor commands.
    #     # The structure [left, right, left, right, 0, 0, 0, 0] matches the
    #     # format required by your motor controller.
    #     motion_data = [
    #         int(left_wheel_rpms),   # Front-Left
    #         int(right_wheel_rpms),  # Front-Right
    #         int(left_wheel_rpms),   # Back-Left
    #         int(right_wheel_rpms),  # Back-Right
    #         0,                      # Extra data slot
    #         0,                      # Extra data slot
    #         0,                      # Extra data slot
    #         0                       # Extra data slot
    #     ]
        
    #     # Assign the list to the message's 'data' field
    #     motion_msg.data = motion_data
    
    #     # Publish the message
    #     self.motion_pub.publish(motion_msg)
        
    # def vel_to_wheel_rpms(self, v, omega):
    #     """Converts linear (v) and angular (omega) velocities to wheel RPMs."""
    #     half_w = self.track_width / 2.0
    #     v_l = v - omega * half_w
    #     v_r = v + omega * half_w
        
    #     rpm_l = (v_l / self.wheel_radius) * 60.0 / (2.0 * math.pi)
    #     rpm_r = (v_r / self.wheel_radius) * 60.0 / (2.0 * math.pi)
        
    #     return rpm_l, rpm_r
    
    def quaternion_to_euler(self,x, y, z, w):

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        #print("current yaw", yaw)
        return roll, pitch, yaw

#--------------------------------
#                gpt

def mod2pi(theta):
    return theta - 2*math.pi * math.floor(theta / (2*math.pi))
def dubins_shortest_path_sample(start, goal, rho, step_size=0.1):
    """
    Pure Python Dubins path sampler (LSL and RSR types for simplicity)
    
    start: (x, y, theta)
    goal:  (x, y, theta)
    rho:   minimum turning radius
    step_size: distance between samples
    
    Returns:
    samples: list of (x, y, theta)
    length: total path length
    """
    sx, sy, syaw = start
    gx, gy, gyaw = goal

    dx = gx - sx
    dy = gy - sy
    D = math.hypot(dx, dy) / rho
    theta = math.atan2(dy, dx)
    alpha = mod2pi(syaw - theta)
    beta = mod2pi(gyaw - theta)

    paths = []

    # LSL
    p_squared = 2 + D**2 - 2*math.cos(alpha - beta) + 2*D*(math.sin(alpha) - math.sin(beta))
    if p_squared >= 0:
        t = mod2pi(-alpha + math.atan2(math.cos(beta) - math.cos(alpha), D + math.sin(alpha) - math.sin(beta)))
        p = math.sqrt(p_squared)
        q = mod2pi(beta - math.atan2(math.cos(beta) - math.cos(alpha), D + math.sin(alpha) - math.sin(beta)))
        paths.append(('LSL', t*rho, p*rho, q*rho))

    # RSR
    p_squared = 2 + D**2 - 2*math.cos(alpha - beta) + 2*D*(-math.sin(alpha) + math.sin(beta))
    if p_squared >= 0:
        t = mod2pi(alpha - math.atan2(math.cos(alpha) - math.cos(beta), D - math.sin(alpha) + math.sin(beta)))
        p = math.sqrt(p_squared)
        q = mod2pi(-beta + math.atan2(math.cos(alpha) - math.cos(beta), D - math.sin(alpha) + math.sin(beta)))
        paths.append(('RSR', t*rho, p*rho, q*rho))

    if not paths:
        raise ValueError("No valid Dubins path found.")

    # Choose shortest
    path_type, t_len, p_len, q_len = min(paths, key=lambda x: x[1]+x[2]+x[3])
    total_length = t_len + p_len + q_len

    # Sample points along path
    samples = []
    n_steps = int(total_length / step_size) + 1
    for i in range(n_steps):
        s = min(i*step_size, total_length)
        # Linear interpolation for simplicity; full Dubins formulas can replace this
        ratio = s / total_length
        x = sx + (gx - sx) * ratio
        y = sy + (gy - sy) * ratio
        theta_interp = mod2pi(syaw + ratio * mod2pi(gyaw - syaw))
        samples.append((x, y, theta_interp))

    return np.array(samples), total_length

# ------------------------------
# Replacement PathMakerFunction
# ------------------------------


# --- MAIN EXECUTION ---
def main(args=None):
    rclpy.init(args=args)
    goal_Arg = sys.argv[1:]
    pure_pursuit_node = PurePursuit(goal_Arg)
    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pure_pursuit_node.get_logger().info('Keyboard interrupt, stopping rover...')
        pure_pursuit_node.stop_rover()
    finally:
        pure_pursuit_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



