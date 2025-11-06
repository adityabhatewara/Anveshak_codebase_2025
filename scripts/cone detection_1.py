#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import math
from nav_msgs.msg import Odometry
class ConeFollower(Node):
    """
    This node does exactly what you described:
      1. Reads YOLO detections of the cone (pixel coords).
      2. Reads ZED camera depth image.
      3. Rotates rover until cone center is at image center (x direction).
      4. Then drives straight until the cone is close enough.
    Everything is done in pixel space — no need for fx, cx, or coordinate transforms.
    """

    def __init__(self):
        super().__init__('cone_follower')

        # ------------- CAMERA PARAMETERS -------------
        # Define your image resolution (ZED RGB/depth image size)
        self.image_width = 1280
        self.image_height = 720
        self.angle_field_of_view = 180
        self.angle_to_turn = 0.0
        self.angle_found = False
        self.initial_yaw = 0.0
        self.initial_yaw_found = False
        self.depth = 0.0
        self.x_err = 0.0
        # Compute image center — our goal is to align cone x_center to this
        self.image_center_x = self.image_width / 2
        self.angle_limit = 0.1

        # ------------- CONTROL PARAMETERS -------------
        self.pixel_tolerance = 40      # pixels (when |x_err| < this, we consider cone centered)
        self.stop_distance = 0.8       # meters (stop when cone is this close)
        self.k_ang = 0.005             # angular velocity gain (rotation speed per pixel offset)
        self.k_lin = 0.4               # linear velocity gain (forward speed)
        self.max_ang = 0.8             # max angular velocity (rad/s)
        self.max_lin = 0.6             # max linear velocity (m/s)

        # ------------- INTERNAL STATE -------------
        self.bridge = CvBridge()       # converts ROS Image -> OpenCV
        self.depth_image = None        # stores latest ZED depth frame
        self.target_bbox = None        # stores latest YOLO detection [x_center, y_center, width, height, conf]
        self.state = "ROTATE"          # 'ROTATE' -> 'FORWARD' -> 'STOP'

        # ------------- ROS COMMUNICATION -------------
        # Subscribe to the ZED depth image topic
        # Each pixel holds the distance (in meters) to whatever is in front of the camera.
        self.create_subscription(Image, "/zed/depth/image_rect_raw", self.depth_callback, 5)

        # Subscribe to YOLO bounding box center (Float32MultiArray [x_center, y_center, width, height, conf])
        self.create_subscription(Float32MultiArray, "/cone_bbox", self.bbox_callback, 5)

        # Publish velocity commands to /motion (Float32MultiArray [linear_vel, angular_vel])
        # The drive node already listens to this topic.
        self.motion_pub = self.create_publisher(Float32MultiArray, "/motion", 5)
        
        # To run based on angle: backup in case pixel fails
        #self.create_subscription(Odometry, "zed/odom", self.odom_callback, 5) #put correct topic for odometry

        # Run control loop at 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("🚀 Pixel-based ConeFollower node started!")

    # -----------------------------------------------------------------
    # 🔹 DEPTH CALLBACK
    # This runs every time the ZED publishes a new depth image.
    # We convert it from ROS2 Image -> OpenCV format and store it.
    # Each pixel stores the distance (float32, meters) to the visible object.
    # -----------------------------------------------------------------
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    # -----------------------------------------------------------------
    # 🔹 YOLO DETECTION CALLBACK
    # This runs whenever YOLO publishes a new bounding box.
    # We assume it's a Float32MultiArray with [x_center, y_center, width, height, conf].
    # We just store it to use in the main control loop.
    # -----------------------------------------------------------------
    def bbox_callback(self, msg):
        if len(msg.data) < 5:
            return
        x, y, w, h, conf = msg.data
        if conf > 0.4:  # skip low-confidence detections
            self.target_bbox = (int(x), int(y))
        else:
            self.target_bbox = None
            self.angle_found = False
            self.initial_yaw_found = False

    # -----------------------------------------------------------------
    # 🔹 MAIN CONTROL LOOP
    # Runs every 0.05 s (20 Hz).
    # 1. Get cone center pixel from YOLO.
    # 2. Get depth at that pixel from ZED depth map.
    # 3. If cone is off-center → rotate until centered.
    # 4. If cone is centered → move forward until depth < stop_distance.
    # 5. Publish velocity commands [linear, angular].
    # -----------------------------------------------------------------
    def control_loop(self):
        if self.target_bbox is None or self.depth_image is None:
            # Nothing detected or no depth image available
            return

        x_center, y_center = self.target_bbox

        # --- Step 1: Get depth (distance) from ZED depth image ---
        # Make sure (x,y) pixel is valid
        if (y_center < 2 or y_center >= self.depth_image.shape[0] - 2 or
            x_center < 2 or x_center >= self.depth_image.shape[1] - 2):
            return

        # Extract a small patch (5x5) around the cone center for a stable median depth
        patch = self.depth_image[y_center-2:y_center+3, x_center-2:x_center+3].flatten()

        # Filter out invalid values (NaN, 0, or inf)
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]
        if len(patch) == 0:
            return

        depth = float(np.median(patch))  # median depth (meters)
        self.depth = depth

        # --- Step 2: Compute pixel error ---
        # Positive if cone is to the RIGHT of image center
        # Negative if cone is to the LEFT
        x_err = x_center - self.image_center_x
        self.x_err = x_err
        

        # --- Step 3: Decide what to do ---
        msg = Float32MultiArray()
        
        if abs(x_err) > self.pixel_tolerance:
            self.state = "ROTATE"
        if self.state == "ROTATE":
            # If cone is not centered horizontally
            if abs(x_err) > self.pixel_tolerance:
                # Rotate proportionally to pixel error
                omega = -self.k_ang * x_err   # negative sign: right cone → rotate right
                omega = max(-self.max_ang, min(self.max_ang, omega))
                vel = 0.0
                self.get_logger().info(f"🔁 Rotating | x_err={x_err:.1f}px ω={omega:.2f}")
            else:
                # Cone roughly centered → switch to forward state
                self.state = "FORWARD"
                vel = 0.0
                omega = 0.0
                self.get_logger().info("✅ Cone centered, switching to FORWARD mode.")

        elif self.state == "FORWARD":
            # Move forward until depth (distance to cone) <= stop threshold
            if depth > self.stop_distance:
                vel = self.k_lin * (depth - self.stop_distance)
                vel = min(self.max_lin, vel)
                omega = 0.0
                self.get_logger().info(f"🚗 Moving forward | depth={depth:.2f} m vel={vel:.2f}")
            else:
                self.state = "STOP"
                vel = 0.0
                omega = 0.0
                self.get_logger().info("🛑 Stopping — reached cone.")

        else:  # STOP
            vel = 0.0
            omega = 0.0
            
        # --- Step 4: Publish motion command ---
        msg.data = [vel, omega]
        self.motion_pub.publish(msg)
    
    def odom_callback(self, msg: Odometry):
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if self.depth == 0.0:
            return

        # Compute angle_to_turn once when cone detected
        if not self.angle_found and abs(self.x_err) > self.pixel_tolerance:
            self.angle_to_turn = (self.x_err / self.image_center_x) * (self.angle_field_of_view / 2)
            self.angle_to_turn = math.radians(self.angle_to_turn)
            self.angle_found = True

        # Store initial yaw once
        if not self.initial_yaw_found:
            self.initial_yaw = yaw
            self.initial_yaw_found = True

        # Current yaw change
        angle = yaw - self.initial_yaw
        angle = math.atan2(math.sin(angle), math.cos(angle))  # normalize

        # Compute angular error
        ang_err = self.angle_to_turn - angle

        # --- State logic ---
        #Bascially Same as the previous case, with angle instead
        if abs(ang_err) > self.angle_limit:
            self.state = "ROTATE"

        msg_out = Float32MultiArray()

        if self.state == "ROTATE":
            if abs(ang_err) > self.angle_limit:
                omega = self.k_ang * ang_err
                omega = max(-self.max_ang, min(self.max_ang, omega))
                vel = 0.0
                self.get_logger().info(f"🔁 Rotating | ang_err={math.degrees(ang_err):.2f} ω={omega:.2f}")
            else:
                self.state = "FORWARD"
                vel = 0.0
                omega = 0.0
                self.get_logger().info("✅ Angle aligned, switching to FORWARD mode.")

        elif self.state == "FORWARD":
            if self.depth > self.stop_distance:
                vel = self.k_lin * (self.depth - self.stop_distance)
                vel = min(self.max_lin, vel)
                omega = 0.0
                self.get_logger().info(f"🚗 Moving forward | depth={self.depth:.2f} m vel={vel:.2f}")
            else:
                self.state = "STOP"
                vel = 0.0
                omega = 0.0
                self.get_logger().info("🛑 Stopping — reached cone.")

        else:  # STOP
            vel = 0.0
            omega = 0.0

        msg_out.data = [vel, omega]
        self.motion_pub.publish(msg_out)

        
    
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ConeFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
