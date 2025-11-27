#!/usr/bin/env python3
import rclpy as r
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import csv
from time import sleep
import math
import numpy as np
from cv_bridge import CvBridge


class AutonomousControl(Node):
    def __init__(self):
        super().__init__("autonomous_control_node")
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.twist_pub = self.create_publisher(Twist, "/motion", self.qos)
        self.rot_pub = self.create_publisher(Int8, "/rot", self.qos)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, self.qos)
        self.yolo_sub = self.create_subscription(Float32MultiArray, "/cone_bbox", self.yolo_callback, self.qos)
        self.state_sub = self.create_subscription(Bool, "/state", self.state_callback, self.qos)
        self.depth_sub = self.create_subscription(Image, "/zed/depth/image_rect_raw", self.depth_callback, self.qos)

        # Define the csv stuff, we will read from kavin.csv
        self.odom_csv_file = "kavin.csv"
        self.error_thresh = 0.5
        self.Kp = 15                                # Remember to tune this value
        self.loaf_velocity = 30                     # Remember to change this value
        self.yolo_msg = None
        self.goal_thresh = 0.5
        self.check = 0
        self.distance = 0.0
        self.depth_image = None
        self.Kp_ang = 10                            # Remember to tune this value
        self.center_align_threshold = 5             # 5 Pixels of threshold
        self.image_width = 1280                     # Check once 
        self.state = False
        
        self.current_orientation = 0.0
        self.desired_orientation = 0.0
        self.aligned = False
        self.aligned_rover = False
        self.current_x = 0.0
        self.current_y = 0.0  
        self.bounding_box_data = []
        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)
        

    def state_callback(self, state: Bool):
        self.state = state.data
        
    
    def depth_callback(self, depth_image: Image):
        if depth_image is None:
            self.get_logger().warn("ZED is not giving depth image")
            return
        
        try: 
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Exception in getting depth image: {e}")


    def yolo_callback(self, msg: Float32MultiArray):
        if msg is None or self.state:
            self.yolo_msg = None
            self.get_logger().info("YOLO topic is still not publishing") 
            return
        
        bounding_box_data = msg.data

        if bounding_box_data is None or len(bounding_box_data) < 4:
            return
        
        self.bounding_box_data = bounding_box_data


    def odom_callback(self, odom: Odometry):
        # From this we can get our current position, or as required by your guys go-to-goal logic
        if odom is not None:
            self.current_x = odom.pose.pose.position.x
            self.current_y = odom.pose.pose.position.y
            self.current_orientation = odom.pose.pose.orientation.z


    def read_from_csv(self):
        with open(self.odom_csv_file, 'r') as f:
            reader = csv.DictReader(f)
            self.csv_data = list(reader)

        if not self.csv_data:
            self.get_logger().warn('No CSV data loaded')
            return 

        curr_distance = float('inf')
        closest_object = {}
        for row in self.csv_data:
            distance = math.dist([self.current_x, self.current_y], [float(row["odom_x"]), float(row["odom_y"])])
            if distance < curr_distance:
                curr_distance = distance
                closest_object = row

        return [float(coord) for coord in closest_object.values()]
    

    def get_depth(self):
        # Get the depth from the detected bounding box
        if self.depth_image is None:
            return

        x_center, y_center = int(self.bounding_box_data[0]), int(self.bounding_box_data[1])

        # Make sure (x,y) pixel is valid
        if (y_center < 2 or y_center >= self.depth_image.shape[0] - 2 or
            x_center < 2 or x_center >= self.depth_image.shape[1] - 2):
            return 

        # Extract a small patch (5x5) around the cone center for a stable median depth
        patch = self.depth_image[y_center - 2:y_center + 3, x_center - 2:x_center + 3].flatten()

        # Filter out invalid values (NaN, 0, or inf)
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]

        if len(patch) == 0:
            return 

        depth = float(np.median(patch))  # median depth (meters)
        return depth
    

    def align_rover(self):
        # X axis offset of the object
        # If the object is on the left of the central line, then positive velocity
        object_offset = (self.image_width / 2) - self.bounding_box_data[0]

        # Make the angular velocity to rotate by
        self.get_logger().info("Sending rotate command to rover to align with the cone")
        if abs(object_offset) > self.center_align_threshold:
            twist_message = Twist()
            twist_message.angular.z = self.Kp_ang * object_offset 
            self.twist_pub.publish(twist_message)
        else:
            self.aligned_rover = True

        
    def timer_callback(self):
        # Main loop begins when the operator first clicks the A button
        # Now we begin our journey towards the delivery location of the corresponding cone

        # First check if the rover is in autonomous mode or not
        if not self.state:
            self.get_logger().warn("Rover is in manual mode, delivery operation will begin in autonomous mode")
            self.aligned = False
            self.check = 0
            return

        # Entered here -> Autonomous mode
        # Get the orientation right now
        odom = self.read_from_csv()
        if self.check == 0 and odom is not None:
            self.desired_orientation, self.object_x, self.object_y = odom # We are extracting the orientation and odometry location of the object
            self.check = 1

        # Having gotten the orientation, we shall rotate until we are at this orientation only
        twist_message = Twist()
        
        if (abs((error := (self.desired_orientation - self.current_orientation))) > self.error_thresh and not self.aligned) or (abs((error := (self.desired_orientation - self.current_orientation))) > self.error_thresh * 2.0 and self.aligned):
            # Orientation is angle after all
            # Error is greater than our threshold, so perform the rotate-in-place
            self.rot_pub.publish(1)  # -> Makes the rover steer to rotate in place
            sleep(2)                 # Sleeping for 2 seconds, assuming it takes this much time to align its wheels
            
            self.get_logger().info("Rotating in place until desired orientation has been achieved")
            # Could use a PID controller here, but P is fine for now
            self.rotate_in_place_velocity = self.Kp * error

            # Publish this autonomous omega now
            twist_message.angular.z = self.rotate_in_place_velocity
            self.twist_pub.publish(twist_message)

        else:
            self.aligned = True
            
            # Alright now i think we are facing the cone
            self.twist_pub.publish(Twist()) # stop rotation
            self.get_logger().info("Aligning the wheels straight")
            self.rot_pub.publish(2)         # -> Aligns rover straight
            sleep(2)                        # Assuming it takes 2s to align
            self.rot_pub.publish(0)
            
            # Rover is facing the right direction now, so now we proceed in this direction
            if self.yolo_msg is None:
                twist_message.linear.x = self.loaf_velocity
                self.twist_pub.publish(twist_message)
                # Loaf velocity is only passed when yolo hasnt detected
            else:
                # So now yolo must have detected and hence time to act on it

                # Align the rover with the object
                if not self.aligned_rover:
                    self.align_rover()           

                self.get_logger().info("Rover is aligned and facing the cone....")

                # Get the depth of the object
                depth = self.get_depth()

                if depth is None:
                    raise TypeError("For some bizarre reason, depth is None")
                
                # Proceed onwards
                twist_message.linear.x = self.loaf_velocity


                # If while going to the cone we deflect, set aligned_rover to False
                if abs(self.bounding_box_data[0] - (self.image_width / 2)) > self.center_align_threshold * 5:
                    self.aligned_rover = False 

                if depth < self.goal_thresh:
                    self.get_logger().info("Reached delivery location!") # Operator should press A for manual mode over here
                    twist_message.linear.x = 0.0
                    self.twist_pub.publish(twist_message)
                    return
                
                self.twist_pub.publish(twist_message)
                # if self.distance < 1.0:
                #     self.get_logger().info("Reached delivery location!") # Operator should press A for manual mode over here
                #     with open(self.odom_csv_file, 'w', newline='') as f:
                #         if self.csv_data:  # Check if there's any data
                #             writer = csv.DictWriter(f, fieldnames=self.reader.fieldnames)
                #             writer.writeheader()
                #             # Skip the current, just finished data row, write the rest
                #             writer.writerows(self.csv_data[:self.row_num])
                #             if self.row_num < len(self.csv_data - 1):
                #                 writer.writerows(self.csv_data[self.row_num+1:])
                #         else:
                #             self.get_logger().warn('No CSV data to write')
                            
                #     return


def main(args=None):
    r.init(args=args)
    node = AutonomousControl()
    try:
        r.spin(node)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        r.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
    