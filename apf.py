#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

DT = 0.1
PREDICT_TIME = 3.0
MAX_SPEED = 1.0
MIN_SPEED = 0.3
MAX_YAWRATE = 1.0
MAX_ACCEL = 0.2
MAX_DYAWRATE = 1.0

# Cost function weights
attraction_factor = 20.0
repulsion_factor = 5.0
max_repulsion = 8.0
# Static goal
goal = np.array([5.0, 6.0])  # Change as needed

class APFController(Node):
    def __init__(self):
        super().__init__('apf_controller')
        self.cmd_pub = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/bcr_bot/scan', self.dist_from_obstacle_callback, 10)
        self.update_counter = 0
        self.obstacle_pts = []
        self.last_v = 0.0
        self.last_w = 0.0
        self.state = np.array([0.0, 0.0, 0.0])  # x, y, yaw
        self.v = 0.0
        self.w = 0.0
        self.timer = self.create_timer(DT, self.timer_callback)

        self.get_logger().info("APF controller running with dynamic obstacle avoidance")

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.state[0] = pose.position.x
        self.state[1] = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        self.state[2] = yaw
        self.v = twist.linear.x
        self.w = twist.angular.z

    def dist_from_obstacle_callback(self, msg: LaserScan):
        self.obstacle_pts = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and r < 0.7:  # filter far-away noise
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                self.obstacle_pts.append((x, y))
        print(self.obstacle_pts)

    def timer_callback(self):
        distance_to_goal = np.hypot(goal[0] - self.state[0], goal[1] - self.state[1])
        if distance_to_goal < 0.1:
            self.get_logger().info("Goal reached!")
            self.publish_stop()
            rclpy.shutdown()
            return
        
        u= self.apf_control()
        cmd = Twist()
        cmd.linear.x = u[0]
        cmd.angular.z = u[1]
        self.cmd_pub.publish(cmd)

    def apf_control(self):
        # Attractive force
        F_att = attraction_factor * self.attractive_force()
        
        # Only consider repulsion if obstacles are nearby
        if self.obstacle_pts:
            F_rep = repulsion_factor * self.repulsive_forces()
        else:
            F_rep = np.array([0.0, 0.0])
        
        # Net force
        F = F_att + F_rep

        self.update_counter += 1
        if self.update_counter % 3 != 0:
            return [self.last_v, self.last_w]

        # If net force is tiny (forces canceled), fallback to goal direction
        if np.linalg.norm(F) < 1e-6:
            dx = goal[0] - self.state[0]
            dy = goal[1] - self.state[1]
            movement_theta = math.atan2(dy, dx)
        else:
            movement_theta = math.atan2(F[1], F[0])

        # Heading error
        turn = movement_theta - self.state[2]
        turn = math.atan2(math.sin(turn), math.cos(turn))

        # Forward speed: uniform
        v = MAX_SPEED

        # Angular speed: proportional with deadband
        if abs(turn) < 0.05:  # small deadband to prevent residual spinning
            w = 0.0
        else:
            w = 1.0 * turn

        self.last_v = v
        self.last_w = w

        return [v, w]


    def motion(self, state, v, w):
        x, y, theta = state
        x += v * np.cos(theta) * DT
        y += v * np.sin(theta) * DT
        theta += w * DT
        return np.array([x, y, theta])

    def attractive_force(self):
        dx = goal[0] - self.state[0]
        dy = goal[1] - self.state[1]
        distance = np.hypot(dx, dy) +1e-6
        return np.array([dx/distance, dy/distance])*distance

    def repulsive_forces(self):
        if not self.obstacle_pts:
            return np.array([0.0, 0.0])

        min_dist = 0.3
        F_obs = np.array([0.0, 0.0])
        for ox_r, oy_r in self.obstacle_pts:
            yaw = self.state[2]
            # Transform from robot to world frame
            ox_w = ox_r * math.cos(yaw) - oy_r * math.sin(yaw) + self.state[0]
            oy_w = ox_r * math.sin(yaw) + oy_r * math.cos(yaw) + self.state[1]
            dx = self.state[0] - ox_w
            dy = self.state[1] - oy_w
            dist = np.hypot(dx, dy)
            strength = min(max_repulsion, 1.0/(dist**2))
            if dist < min_dist:
                F_obs+= 100 * np.array([dx, dy])  # collision can happen, discourage this path as much as possible
            else:
                F_obs += strength * (np.array([dx, dy]))
        return F_obs  # closer is worse (more the distance, lower the cost)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = APFController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
