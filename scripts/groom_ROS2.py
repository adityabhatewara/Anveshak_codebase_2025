### Work in progress ### 

#!/usr/bin/env python3

import rclpy as r
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray, Bool, Float32MultiArray, Int8
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from traversal2.msg import WheelRpm    #type: ignore
import math
import time 
from collections import defaultdict
import random
from csv import writer
from ultralytics import YOLO

class Groom(Node):
    def __init__(self):
        super().__init__("haha_groom_node") # Node Initialization


        self.bridge = CvBridge()            # CV2 Bridge class instance
        self.count_arrows = 0               # Number of arrows found so far
        self.total_number_of_arrows = 5     # Total number of arrows that will in the competition
        self.state = False                  # Current state of the rover
        self.gps_called = 0                 
        self.rot = 0            
        self.recieved_image = False         # Whether an image was recieved from ZED camera
        self.zed_camera_angle = 0           # Current angle of ZED camera, which changes while performing search
        self.captured_initial_yaw = False   # Whether the initial yaw of the ZED camera was captured
        self.initial_yaw = 0                # Initial yaw of ZED camera
        self.pls_call_rot_once = 0
        self.arrow_detected = False         # Whether an arrow was detected
        self.init = False
        self.turn = False                   # Whether the rover is turning or not
        self.search_called = False          # Whether search was called
        self.time_thresh = 20               # Amount of time before search is called again, in seconds
        self.start_time = time.time()       # Start time to keep track of when to perform search
        self.cone_detected = False          # Whether cone was detected
        self.direction = "None"
        self.reached_arrow_distance = 1.5   # How close we should get to the arrow
        self.angle_thresh = 4               # Tolerance in angle of turning of the ZED camera
        self.start_angle = 50               # Starting angle of the ZED camera
        self.base_rot_direction = 1         # Multiplier to decide which direction to rotate the camera
        self.base_rotation_index = 1        # The index to which the pwm value should be sent to
        self.angles_dict = defaultdict(list)# Its made a default dict so that if any key that doesnt exist is called, it by default returns zero
        self.angle_rover_align = 0          # The angle by which to rotate the rover in order to align it with the closest arrow
        self.Kp = 1.0                       # Kp for driving straight
        self.initial_drift_angle = 0        # As the name suggests

        self.arrow_model = YOLO("/some/path/best.pt")     # Arrow model -> Remember to change the path to the file.
        self.cone_model = YOLO("/some/path/best.pt")      # Cone model -> Remember to change the path to the file

        self.arrow_classes = self.arrow_model.names

        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        ###### Subscribers #######
        # To get the current state of the rover
        self.state_subscriber = self.create_subscription(
            Bool,
            "/state",
            self.state_callback,
            self.qos)

        # To get ZED imu
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "rtabmap_topic",
            self.odom_callback,
            self.qos)

        # Still unsure about this
        self.encoder_arm_subscriber = self.create_subscription(
            Float32MultiArray,
            "/enc_arm",
            self.enc_callback,
            self.qos)

        # To get the gps of the rover
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "/gps_coordinates",
            self.gps_callback,
            self.qos)

        # ZED colour image subscriber
        self.rgb_image_subscriber = self.create_subscription(
            Image,
            "/zed2i/zed_node/rgb/image_rect_color",
            self.color_img_callback,
            self.qos)

        # ZED depth image subscriber
        self.depth_image_subscriber = self.create_subscription(
            Image,
            "/zed2i/zed_node/depth/depth_registered",
            self.depth_img_callback,
            self.qos)

        ###### Publishers #######
        # Publish to /stm_write the pwm
        self.pwm_pub = self.create_publisher(Int32MultiArray, "/stm_write", self.qos)
        # Publish the WheelRpm message
        self.wheelrpm_pub = self.create_publisher(WheelRpm, "/motion", self.qos)
        # Not sure
        self.rotin_pub = self.create_publisher(Int8, "/rot", self.qos)
        # Not sure
        self.gps_pub = self.create_publisher(Int8, "/gps_bool", self.qos)

        ##### Timer #####
        self.timer = self.create_timer(0.1, self.timer_callback)

    @property
    def nonsensically_long_string(self):
        # Defined property cause it looks cool
        return "=" * random.randint(20, 30)
    

    def state_callback(self, state: Bool):
        self.state = state

    
    def odom_callback(self, odom: Odometry):
        quaternion = odom.pose.pose.orientation

        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w] 
        current_yaw = math.degrees(euler_from_quaternion(quaternion_list)[2])
        normalized_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))
        self.get_logger().info(f"Capturing current yaw: {normalized_yaw}")
        self.zed_camera_angle = normalized_yaw      


    def enc_callback(self, enc: Float32MultiArray):
        self.enc_data = enc.data[1]


    def gps_callback(self, gps: NavSatFix):
        if gps.latitude and gps.longitude:
            self.current_latitude = gps.latitude
            self.current_longitude = gps.longitude


    def color_img_callback(self, img: Image):
        try:
            color_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            self.color_image = color_image[30:330, 170:490]
            self.recieved_image = True
        except Exception as e:
            self.get_logger().error(f"Color image retrieval failed with error {e}")
        

    def depth_img_callback(self, img: Image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
            self.depth_image = depth_image[30: 330, 170: 490]
        except Exception as e:
            self.get_logger().error(f"Depth image retrieval failed with error {e}")


    # def _rotate_the_camera(self, direction: int):
    #     '''
    #     Helper function that just publishes the rotation pwm for the camera

    #     :param direction: 1 for counter-clockwise, -1 for clockwise rotation
    #     :type direction: int
    #     '''

    #     pwm = Int32MultiArray()
    #     pwm.data = [255 * direction if i == self.base_rotation_index else 0 for i in range(6)]
    #     time.sleep(0.1)
    #     self.pwm_pub.publish(pwm)

    
    def _stop_rover(self):
        '''
        Helper function which stops the rover
        '''
        self.get_logger().info(self.nonsensically_long_string + " Stopping the Rover " + self.nonsensically_long_string)
        self.wheelrpm_pub.publish(WheelRpm())

        pwm = Int32MultiArray()
        pwm.data = [0, 0, 0, 0, 0, 0]
        self.pwm_pub.publish(pwm)
        

    def log_gps(self):
        '''
        This function calculates a good estimate of current gps coordinates by sampling a 100 times 
        and taking average
        '''

        self.get_logger().info("Writing the gps coordinates to a csv file.....")
        
        # Note the gps coordinates of the arrows
        # To get a better estimate, we are taking the average of 100 values
        latitude_sum, longitude_sum = 0, 0
        for _ in range(100):
            self.gps_pub.publish(self.gps_called)
            latitude_sum += self.current_latitude
            longitude_sum += self.current_longitude
            self.get_logger().info(self.nonsensically_long_string + "Calculated sum" + self.nonsensically_long_string)
            self.get_logger().info(f"latitude sum: {latitude_sum} & longitude sum: {longitude_sum}\n" + self.nonsensically_long_string)
            self.get_logger().info("Sleeping for 10s...") 
            time.sleep(10)

            # Write to a text file and a csv file
            average_latitude = latitude_sum / 100
            average_longitude = longitude_sum / 100
            with open("coordinates.txt", "w") as coords_file:
                coords_file.write(f"Latitude: {average_latitude} Longitude: {average_longitude}")

            with open("sparsh2.csv", "a", newline="") as csv_file:
                csv_writer = writer(csv_file)
                csv_writer.writerow([average_latitude, average_longitude])


    def move_straight(self):
        '''
        This function coincidently moves the rover straight. Gasp.
        '''
        # Create a new WheelRpm message
        msg = WheelRpm()


        
    def v1_competition(self):
        '''
        Function that prompts the rover back to manual mode and ends our run
        '''
        # Stop the rover
        pass


    def _normalize_angle(self, angle):
        '''
        Normalize the angle in some weird way
        
        :param angle: The angle to be normalized
        :return normalized_angle: The normalized angle
        '''
        normalized_angle = angle
        if angle > 120:
            normalized_angle -= 360
        elif angle < -120:
            normalized_angle += 360

        return normalized_angle
    

    def _rotation(self, direction, mode):
        '''
        Rotation helper function that reduces redundancy

        :param mode: The type of rotation being performed. 1 -> Rotinplace & 2 -> Rotate
        :param direction: The direction of rotation. -1 -> Clockwise (Right) & 1 -> Anticlockwise (Left)
        :return None:  
        '''
        # Create the WheelRpm message
        msg = WheelRpm()

        # Calculate the difference in angle
        difference = self.zed_camera_angle - self.initial_yaw
        difference = self._normalize_angle(difference)
        error = self.angle_rover_align - abs(difference)

        # Visualization Spam
        self.get_logger().info(self.nonsensically_long_string + " INFO " + self.nonsensically_long_string)
        self.get_logger().info(f"Type of rotation: {'Rotate' if mode == 2 else 'Rotinplace'}")
        self.get_logger().info(f"Difference: {difference}")
        self.get_logger().info(f"Rotation angle: {self.angle_rover_align}")
        self.get_logger().info(f"Initial yaw: {self.initial_yaw}")
        self.get_logger().info(f"Current z angle: {self.zed_camera_angle}")
        self.get_logger().info(f"Error: {error}")
        self.get_logger().info(2 * self.nonsensically_long_string)

        if abs(error) >= 0.5 * self.angle_thresh:
            msg.omega = direction * mode * 40
            self.get_logger().info(f"Rotating in direction: {direction} with the error angle: {error}")

        else:
            if mode == 1:
                # Some rotinplace specific stuff to be done
                self.rot = 2
                self.rotin_pub.publish(self.rot)
                self.rot = 0
                self.rotin_pub.publish(self.rot)
                self.count_arrows += 1
            
            self.get_logger().info(self.nonsensically_long_string + " ROTATION DONE " + self.nonsensically_long_string)

            # Reseting
            self.initial_drift_angle = self.zed_camera_angle
            self.start_time = time.time() - 10
            self.initial_yaw = self.zed_camera_angle
            self.turn = False
            self.direction = "None"

        # Publish WheelRpm message
        self.wheelrpm_pub.publish(msg)
    

    def _rotate(self, direction: int):
        '''
        Function to rotate the rover

        :param direction: Direction of rotation. -1 -> Clockwise (Right) & 1 -> Anticlockwise (Left)
        '''
        self.get_logger().info(self.nonsensically_long_string + "Rotating...." + self.nonsensically_long_string)

        # Call private helper function
        self._rotation(direction=direction, mode=2)


    def _rot_in_place(self, direction: int):
        '''
        Function to rotate the rover in place

        :param direction: Direction of rotation. -1 -> Clockwise (Right) & 1 -> Anticlockwise (Left)
        '''
        self.get_logger().info("Stopping rover, preparing to ROTATE_IN_PLACE")
        self._stop_rover()

        # IDK what this is but sure
        if self.pls_call_rot_once:
            self.rot = 1
            self.rotin_pub.publish(self.rot)
            self.pls_call_rot_once = False

        # Printing time cause why not
        self.get_logger().info(f"Current time: {time.time()}, Starting time: {self.start_time}")

        # Call rotation helper function
        self._rotation(direction=direction, mode=1)


    def perform_mission(self):
        '''
        The function that shows the flow of control in the mission
        '''
        pass


    def timer_callback(self):
        '''
        Main loop
        '''
        pass


def main(args=None):
    r.init(args=args)
    node = Groom()
    try:
        r.spin(node)
    except KeyboardInterrupt:
        print("YOU DARE KeyboardInterrupt ME!!!")
    finally:
        r.shutdown()
        node.destroy_node()
        cv.destroyAllWindows()


if __name__=="__main__":
    main()
