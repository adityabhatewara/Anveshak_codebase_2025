import sys
import math
import rclpy
from rclpy.node import Node

import numpy as np
import math
from tf_transformations import euler_from_quaternion

## Msg data structs imports ##
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import utm

import time

from path_generator import find_shortest_path_to_point





class PurePursuit(Node):
    def __init__(self,goal):
        super().__init__("pure_pursuit_controller")

        # --- ROS Parameters --- #
        self.declare_parameter('goal_lat', goal[0])
        self.declare_parameter('goal_lon', goal[1])
        self.declare_parameter('lookahead_distance', 0.7)
        self.declare_parameter('wheelbase', 1.2)
        self.declare_parameter('wheel_radius_m', 0.25) # 10cm radius
        self.declare_parameter('track_width_m', 1.2) # Distance between left and right wheels
        self.declare_parameter('goal_tolerance_m', 1.0)
        self.declare_parameter('control_frequency', 10.0) # Hz

        # --- State Machine --- #
        self.state = 'WAITING_FOR_POSE'
        self.get_logger().info(f"Node initialized. Current state: {self.state}")

        # --- Class Attributes --- #
        self.pose_odom = None          # Stores (x, y, theta) from odometry
        self.current_lat_lon = None    # Stores (lat, lon) from GPS
        self.path = []
        
        # Get parameters #
        self.goal_gps = [
            self.get_parameter('goal_lat').get_parameter_value().double_value,
            self.get_parameter('goal_lon').get_parameter_value().double_value
        ]
    
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius_m').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width_m').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance_m').get_parameter_value().double_value

        # print("haha")
        # print(self.goal_x,self.goal_y) # uncomment for debugging #

        # --- QoS Profiles --- #
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # --- Publishers --- #
        self.wheelrpm_pub = self.create_publisher(Twist, '/motion', qos_reliable)
        
        # --- Subscribers --- #
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_reliable)
        # self.create_subscription(Imu, '/zed2i/zed_node/imu/data', self.ZEDcallback, qos_best_effort)
        # self.create_subscription(NavSatFix, '/gps_coordinates', self.gps_callback, qos_best_effort)



        self.latitude0 = math.radians(12.99151) #### Hard coded current gps coordinates. Change this ###
        self.longitude0 = math.radians(80.23362)

        self.global_n = math.radians(30.0) ### this has been hardcoded at the moment

        self.EARTHS_RADIUS = 6378137.0

        # --- Control Loop Timer --- #
        timer_period = 1.0 / self.get_parameter('control_frequency').get_parameter_value().double_value
        self.control_timer = self.create_timer(timer_period, self.pp_callback)
    


    # --- SUBSCRIBER CALLBACKS --- #
    def odom_callback(self, msg: Odometry):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        roll, pitch, theta = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.pose_odom = (pos_x, pos_y, theta)

        if self.state == 'WAITING_FOR_POSE':
            self.state = 'GENERATING_PATH'
            self.get_logger().info("Initial odometry received. State changed to GENERATING_PATH.")


    # def ZED_callback(self,data): ## incase we want to use ZED's odometry ##
    #     current_x = data.orientation.x
    #     current_y = data.orientation.y
    #     current_z = data.orientation.z
    #     current_w = data.orientation.w
    #     current_tuple=(current_x, current_y, current_z, current_w)
    #     current_pitch, current_roll, self.z_angle_in_rad = self.euler_from_quaternion(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
    
    # --- Waypoints generating function --- #
    def get_path(self, rho=1.0, step_size=0.1):
        """Generates a dynamic Dubins path from the current pose to the GPS goal."""
        try:
            self.get_logger().info("Now converting gps to xy...")

            # --- conversion of gps to xy --- #
            lat = self.goal_gps[0]
            lon = self.goal_gps[1]

            dlat = lat - self.latitude0
            dlon = lon - self.longitude0
            # print (lat,lon)

            east = self.EARTHS_RADIUS * dlon * math.cos(self.latitude0)
            north = self.EARTHS_RADIUS * dlat

            self.goal_x = east * math.sin(self.global_n) + north * math.cos(self.global_n)
            self.goal_y = -east * math.cos(self.global_n) + north * math.sin(self.global_n)
            
            

            current_odom_x, current_odom_y, _ = self.pose_odom
            goal_xy_in_odom = (self.goal_x, self.goal_y)

            self.get_logger().info("Goal coordinates coverted to xy. Now generating dubin's path...")
            print(f"Goal pose in xy wrt to rover:{goal_xy_in_odom}")

            path_x, path_y, path_yaw, mode, _ = find_shortest_path_to_point(self.pose_odom[0], self.pose_odom[1], math.degrees(self.pose_odom[2]), goal_xy_in_odom[0],goal_xy_in_odom[1], curvature = 1.0)
            path = np.column_stack((path_x, path_y))
            
            if path is None:
                self.get_logger().warn("PathMakerFunction returned no path.")
                return False
            
            self.path = path
            self.get_logger().info(f"Warpoints recieved...")
            print(self.path)

            return True
        except Exception as e:
            self.get_logger().error(f"Error aagaya🥲 in get_path(): {e}")
            return False

    # --- Function to stop the rover --- #
    def stop_rover(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z= 0.0
        self.wheelrpm_pub.publish(msg)

    # --- to check if goal if reached --- #
    def isGoalReached(self, tolerance=1.0) -> bool:

        """
        Checks if the rover is within a 'tolerance' distance of the final goal.
        
        Args:
            tolerance (float): The maximum allowed distance (in meters) from the goal.
        
        Returns:
            bool: True if the goal is reached, False otherwise.
        """

        current_x, current_y, _ = self.pose_odom

        distance_to_goal = math.hypot(self.goal_x - current_x, self.goal_y - current_y)
        print(f"goal = {self.goal_x},{self.goal_y}.............check dist: {distance_to_goal}")
        if distance_to_goal <= tolerance:
            print("reached")
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
            if self.isGoalReached():
                self.get_logger().info("pauch gaya.")
                self.state = 'GOAL_REACHED'
                self.stop_rover()
            else:
                linear_velocity, angular_velocity = self.pure_pursuit_control()
                # left_rpm, right_rpm = self.vel_to_wheel_rpms(linear_velocity, angular_velocity)
                # self.giveRoverPWM(left_rpm, right_rpm)
                print(f"(x,y) = ({self.pose_odom[0]},{self.pose_odom[1]})")
                msg = Twist()
                msg.linear.x = linear_velocity
                msg.angular.z= angular_velocity
                self.wheelrpm_pub.publish(msg)

        
        if self.state == 'GOAL_REACHED':
            self.get_logger().info("Execution complete. Stopping rover.", once=True)
            self.stop_rover()

    # --- Pure pursuit and helper fucntions --- #
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
        r = math.hypot(dx,dy)
        local_goal_x = math.cos(theta) * dx + math.sin(theta) * dy
        local_goal_y = -math.sin(theta) * dx + math.cos(theta) * dy

        # 4. Calculate the curvature of the arc to the goal point
        alpha = math.atan2(local_goal_y, local_goal_x)
        curvature = (2 * math.sin(alpha)) / r

        # 5. Set linear velocity and calculate angular velocity
        linear_velocity = 20.0  # Constant forward speed
        angular_velocity = -1*linear_velocity * curvature
        
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


# --- MAIN EXECUTION --- #
def main(args=None):
    rclpy.init(args=args)
    goal_Arg = sys.argv[1:]
    goal_Arg = [float(goal_Arg[0]),float(goal_Arg[1])]
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


