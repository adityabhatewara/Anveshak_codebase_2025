#!/usr/bin/env python3

from rrt_star_final import RRT
from rrt_star_final import visualize_tree
from get_occupancy_grid import Map_getter
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt

class Nav(Node):
    def __init__(self, x, y):

        super().__init__("feed")
        self.position = self.create_subscription(Odometry, "/bcr_bot/odom", self.pp_callback, 10)
        self.velocity = self.create_publisher(Twist, '/bcr_bot/cmd_vel' , 10)
        
        self.start_pos = (45, 68)
        self.goal_pos = (x, y)
        
        self.step_size = 10.0
        self.search_radius = 20.0
        self.goal_radius = 5.0
        map = Map_getter()
        self.occupancy_grid = map.get_occupancy_grid()
        
        self.reached = False
        self.waypoint_index = 0
        self.just_started = True
        self.max_iterations = 3000   ## CHANGE AS NEEDED
        self.waypoint_wrt_rover = None
        self.path = []
        self.path_sparse = []
        self.get_path()


        # self.path = [(0,0), (1,0), (2,1), (2,2)]
    def get_path(self):
        self.map_size = (self.occupancy_grid.shape[1], self.occupancy_grid.shape[0])
        obj = RRT(
            self.start_pos, 
            self.goal_pos, 
            self.map_size, 
            self.step_size, 
            self.search_radius, 
            self.goal_radius, 
            self.occupancy_grid
            )
        
        self.initial_path = None

        if not obj.is_valid_node(obj.start_node):
            print("start node itself is invalid bro")
            return

        if not obj.is_valid_node(obj.goal_node):
            print('r u dumb, give me a goal thats not obstacle!')
            return
            
        # obj.develop_tree()
        print("iterating now")

        for i in range(self.max_iterations):
            print(f"iteration : {i}")
            obj.develop_tree()

        self.initial_path = obj.path_generator()
        print(self.initial_path)

        self.path = []
        for i in range(len(self.initial_path)):
            x, y = self.initial_path[i]
            self.path_sparse.append((x - 45, y-68))
        print(self.path_sparse)
        # after you generate `self.initial_path` in get_path():
        self.path_in_map = self.densify_path(self.path_sparse, max_segment_length=1.0)
        print(self.path_in_map)
        self.path= self.map_to_world(self.path_in_map)
        print(self.path)

        visualize_tree(obj.node_tree, self.occupancy_grid, obj.goal_node, obj.start_node, self.initial_path)
        plt.show()


        

        # return self.path

    def densify_path(self, path, max_segment_length=1.0):
        """
        Given a list of (x,y) waypoints, return a new list where
        no two consecutive points are more than max_segment_length apart.
        """
        if not path:
            return []

        new_path = []
        for (x1, y1), (x2, y2) in zip(path, path[1:]):
            # always include the first endpoint
            new_path.append((x1, y1))

            # compute segment length and how many subdivisions
            dx, dy = x2 - x1, y2 - y1
            dist = math.hypot(dx, dy)
            if dist <= 0:
                continue
            num_steps = int(math.ceil(dist / max_segment_length))

            # insert intermediate points (excluding the final endpoint)
            for step in range(1, num_steps):
                t = step / num_steps
                xi = x1 + t * dx
                yi = y1 + t * dy
                new_path.append((xi, yi))

        # append the very last waypoint
        new_path.append(path[-1])
        return new_path

    def map_to_world(self,path_in_map_pixels, resolution=0.05, origin=(-15.3, -16.8), downscale=3.0):
        """
        Converts pixel coordinates from a downscaled map to real-world coordinates.
        
        Parameters:
        - path_in_map_pixels: List of (x, y) pixel positions (from downscaled map)
        - resolution: Original map resolution (meters/pixel)
        - origin: Origin from map YAML file (real-world coordinate of bottom-left corner)
        - downscale: Factor by which map was downscaled (e.g., 2.0 means original was twice the size)
        """
        world_path = []
        for px, py in path_in_map_pixels:
            # Scale back up to original pixel coordinates
            original_px = px * downscale
            original_py = py * downscale
            
            # Convert to world coordinates
            wx = original_px * resolution + 0
            wy = original_py * resolution + 0
            world_path.append((wx, wy))
        return world_path



    
    # def gothere_callback(self, msg):
    #     # if self.just_started == True:
    #     #     self.start_pos = msg.pose.pose.position
    #     #     self.just_started = False
    #     cmd = Twist()

    #     if self.waypoint_wrt_rover is None:
    #         self.X, self.Y = self.get_next_waypoint(self.waypoint_index)
    #         print(f"({self.X}, {self.Y})")
    #         # self.X = self.X - 45
    #         # self.Y = self.Y - 68
    #         self.waypoint_index += 1
    #         return
    
    #     dist = self.abs_dist(msg)
        # self.coordinate = msg.pose.pose.position
        # quat = msg.pose.pose.orientation
        # qx = quat.x
        # qy = quat.y
        # qz = quat.z
        # qw = quat.w

        # # Compute yaw (in radians)
        # yaw = math.atan2(2.0 * (qw * qz + qx * qy),
        #                 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # # cmd.linear.x = 0.4 * dist

        # angle_to_goal = np.arctan2(self.Y - self.coordinate.y, self.X - self.coordinate.x)
        # angle_diff = angle_to_goal - yaw
        # angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff)) 
        # cmd.angular.z = angle_diff


    #     if abs(angle_diff)<0.005:
    #         cmd.angular.z= 0.0
    #         cmd.linear.x = dist * 0.5

    #     # else:
    #     #     # integral_error=dist*0.1
    #     #     # derivative_error=(dist-self.prev_error)/0.1
    #     #     cmd.angular.z = 0.0
    #     #     cmd.linear.x=1.0 #+ 0.0*integral_error + 0.0*derivative_error
    #     #     #self.prev_error=distance

    #     if dist < 0.025:
    #         twist = Twist()
    #         self.velocity.publish(twist) 
    #         if not self.reached:
    #             print(dist)
    #             self.reached = True
    #             self.goal_reached()
    #             self.X, self.Y = self.get_next_waypoint(self.waypoint_index)
    #             print(f"({self.X}, {self.Y})")
    #             self.waypoint_index += 1
    #             self.reached = False
    #         return


    #     # print(dist)
    #     self.velocity.publish(cmd)
        

    def goal_reached(self, rover_pose):
        dist = np.sqrt((rover_pose[0]-self.path[-1][0])**2 + (rover_pose[1]-self.path[-1][1])**2)
        print(dist)
        if dist < 0.05:
            print("goal reached")
            return True
        return False

                     
    
    def pp_callback(self, odom_msg: Odometry):
        # Publish as Twist
        cmd = Twist()


        # Extract rover's current pose and heading (theta)
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        # Convert quaternion to euler yaw (theta)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        ## Use built in function in ros2 called euler from quarternion for finding yaw, prolly would be faster

        angle_to_goal = np.arctan2(self.path[1][1], self.path[1][0])
        angle_diff = angle_to_goal - theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff)) 
        
        if self.just_started:
            if abs(angle_diff) > 0.2 :
                cmd.angular.z = angle_diff
                cmd.linear.x = 0.0
                self.velocity.publish(cmd)
                return
            self.just_started = False
        rover_pose = (x, y, theta)
        
        # Choose lookahead_distance and wheelbase as appropriate for your robot
        lookahead_distance = 0.7 # 0.7 meters works
        wheelbase = 0.3            # example: 0.3 meters (distance between wheels)

        # Calculate control commands
        linear_velocity, angular_velocity = self.pure_pursuit_control(
            rover_pose, lookahead_distance, wheelbase
        )

        
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.velocity.publish(cmd)

    def pure_pursuit_control(self, rover_pose, lookahead_distance, wheelbase):
        x, y, theta = rover_pose

        
        # 1. Find closest path point
        (closest_point, closest_index) = self.get_closest_path_point((x, y))

        # 2. Find goal point lookahead distance ahead
        goal_point = self.find_lookahead_point((x, y), lookahead_distance, closest_index)
        goal_x, goal_y = goal_point

        # 3. Transform goal point to rover coordinate frame
        dx = goal_x - x
        dy = goal_y - y
        # rotate by -theta to get coordinates in vehicle frame
        local_goal_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        local_goal_y = math.sin(-theta) * dx + math.cos(-theta) * dy

        # 4. Compute control
        alpha = math.atan2(local_goal_y, local_goal_x)
        curvature = (2 * math.sin(alpha)) / lookahead_distance

        linear_velocity = 0.4  # You can tune this value or make it dynamic
        angular_velocity = linear_velocity * curvature
        # print(f"Rover Pose: ({x:.2f}, {y:.2f}, {theta:.2f})")
        # print(f"Goal (World): ({goal_x:.2f}, {goal_y:.2f})")
        # print(f"Local Goal: ({local_goal_x:.2f}, {local_goal_y:.2f})")
        # print(f"Alpha: {alpha:.2f}, Curvature: {curvature:.2f}")
        
        if self.goal_reached((x,y)):
            return 0.0,0.0
        return linear_velocity, angular_velocity

    def get_closest_path_point(self, rover_xy):
        """
        rover_xy: (x, y)
        path: list of (x, y)
        returns: (closest_point, closest_index)
        """
        closest_index = 0
        min_dist = float('inf')
        rover_x, rover_y = rover_xy
        for i, (px, py) in enumerate(self.path):
            dist = math.hypot(px - rover_x, py - rover_y)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        closest_point = self.path[closest_index]
        return closest_point, closest_index

    def find_lookahead_point(self, rover_xy, lookahead_distance, closest_index):
        """
        rover_xy: (x, y)
        path: list of (x, y)
        lookahead_distance: float
        closest_index: int
        returns: (goal_x, goal_y)
        """
        x_r, y_r = rover_xy
        for i in range(closest_index, len(self.path) - 1):
            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]
            d1 = math.hypot(x1 - x_r, y1 - y_r)
            d2 = math.hypot(x2 - x_r, y2 - y_r)
            if d2 >= lookahead_distance:
                # Interpolate to find exact lookahead point
                segment_len = math.hypot(x2 - x1, y2 - y1)
                if segment_len == 0:
                    return (x2, y2)
                direction = ((x2 - x1) / segment_len, (y2 - y1) / segment_len)
                remaining = lookahead_distance - d1
                goal_x = x1 + direction[0] * remaining
                goal_y = y1 + direction[1] * remaining
                return (goal_x, goal_y)
        # If we run past the path, use the last point
        return self.path[-1]
    


    # def abs_dist(self,msg):
    #     return np.sqrt((2-msg.pose.pose.position.x)**2 + (2 - msg.pose.pose.position.y)**2)



def main(args = None):
    rclpy.init(args=args)
    x = int(input("Enter the x coordinate of the goal:"))
    y = int(input("Enter the y coordinate of the goal:"))
    x = x 
    y = y 
    node = Nav(x, y)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
