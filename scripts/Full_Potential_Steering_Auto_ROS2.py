#!/usr/bin/env python3

import rclpy as r
from rclpy.node import Node
import copy
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Int8, Int32MultiArray, Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import queue
from operator import add 
from rclpy.qos import QoSProfile, ReliabilityPolicy
from time import time, sleep
from rclpy.time import Time
import csv
from rclpy.clock import ClockType

class Drive(Node):

    def __init__(self):
        super().__init__("fullboi")

        self.max_steer_pwm = 127                            # Max steer velocity in a way
        self.init_dir = [1,1,1,-1,1,1,1,1]                 # Dont know what this is -- Nigga this is the motor direction array
        self.state = False                                  # Decides which mode rover is in: True -> Autonomous & False -> Manual

        self.mode_up_button = 5                             # Buttons that cycle through the modes
        self.mode_down_button = 4                           # 0 -> 1 -> 2 -> 3 (up)

        self.fb_axis = 1                                    # To move rover forward-back
        self.lr_axis = 3                                    # To move rover left-right
        self.forward_btn = 3                                # To turn all wheels parallel to chassis
        self.parallel_btn = 1                               # To turn all wheels perpendicular to chassis
        self.rotinplace_btn = 2                         
        self.autonomous_btn = 0                             # Autonomous button switches between manual and autonomous
        self.log_btn = 10                                   # Button to log current gps coordinates 

        self.steer_islocked = True                          # Checks if the steering has been unlocked
        self.steering_ctrl_unlocked = [0, 0]                # Will store the buttons which control when the steering is unlocked
        self.steering_ctrl_pwm = [0, 0]                     # Will store the axis input when the steering is unlocked

        self.full_potential_islocked = True                 # Checks if full potential steering is unlocked
        self.full_potential_pwm = [0, 0, 0, 0]              # Will store the axis input for each of the wheels when full potential is unlocked

        self.steer_unlock_axis = 5                          # The joy axis that locks full potential when the steering is being toggled
        self.steer_samedir_axis = 3                         # Tells which axis will be used for samedir motion
        self.steer_oppdir_axis = 4                          # Tells which axis will be used for rotin place

        self.full_potential_unlock_axis = 2                 # The joy axis that locks steering when full_potential is being toggled
        self.fl_wheel_axis = 1                              # Axis that will give input for front left wheel
        self.fr_wheel_axis = 4                              # Axis that will give input for front right wheel        
        self.bl_wheel_axis = 0                              # Axis that will give input for back left wheel
        self.br_wheel_axis = 3                              # Axis that will give input for back right wheel
        self.change_dir_button = 7
        self.drive_ctrl = [0, 0]                            # Drive fb and lr axes
        self.steering_ctrl_locked = [0, 0, 0]               # Gives configurations for steering (buttons)
        self.curve_opp_str = 0                              # Stores input given by the fourth axis that moves the rover in a curve
        self.crab = False                                   # Crab rotation mode for rover
        self.enc_data = [0.0] * 4                           # Encoder data
        self.mode = 0                                       # Goes from 0 to 4

        # Print parameters
        self.prints_per_iter = 1                            # Idk this is just weird
        self.print_ctrl = self.prints_per_iter

        # Autonomous parameters
        self.crab_rotate = False                            # Crab autonomous
        self.autonomous_vel = 0                             # Velocity of wheels in autonomous mode
        self.autonomous_omega = 0                           # Omega of wheels in autonomous mode
        self.rotin = 0                                      # And need to understand what this is                
        self.state_init = [False, False, False]             # Need to understand this

        self.pwm_msg = Int32MultiArray()                    # PWM message
        self.steering_complete = True
        self.drive_multipliers = [35,70,110,140,180]        # Drive multipliers
        self.s_arr = [self.max_steer_pwm] * 5               # No modes in steering       
        self.initial_enc_data = [0,0,0,0]                   # Encoder Initial Values
        self.kp_steer = 30                       
        self.qsize = 3                                      # Size of the queue for velocity smoothing
        self.vel_prev = queue.Queue(self.qsize)             # To implement ledc type control
        self.omega_prev = queue.Queue(self.qsize)           # To implement ledc type control
        self.start_time = time()                            # Start time to maintain 10s time limit for angle operations
        self.time_thresh = 10                               # A timelimit on how long the steer function can work for
        self.error_thresh = 2.0                             # Angle error threshold in deg
        clock_type = self.get_clock().clock_type
        self.joy_last_timestamp = None
        self.timeout_time = 15.0                             # After 5s of no joy command recieved, cut the rover
        self.IndividualSteer = False                        # Stores if the rover is in individual steering or not
        
        self.in_steer = False                               # Flag variable to check if timer should only perform steering
        self.rotinplace = False                             # Condition if rover is in rotin place alignment  
        self.gps_message = None                             # The current GPS message from the GPS topic
        self.gps_file_name= "adeesh.csv"                    # CSV file in which we will log the GPS coords during recon
        with open(self.gps_file_name, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["Latitude", "Longitude"])
        
        self.object_number = 1                              # Number of objects found in Recon
        self.change_dir = False
        self.actual_directiom = "ZedForward" if not self.change_dir else "ArmForward" 

        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        ##### Subscribers #####
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, self.qos)
        # self.enc_sub = self.create_subscription(Float32MultiArray, "/enc_auto", self.enc_callback, self.qos)
        self.rpm_sub = self.create_subscription(Twist, "/motion", self.autonomous_callback, self.qos)
        self.rot_sub = self.create_subscription(Int8, "/rot", self.rotinplace_callback, self.qos)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, self.qos)
        # Remember to change the GPS topic
        ##### Publishers #####
        self.pwm_pub = self.create_publisher(Int32MultiArray, "/motor_pwm", self.qos)
        self.state_pub = self.create_publisher(Bool, "/state", self.qos) #NIGGA DONT COMMENT THIS OUT LOL
        self.IndSteer_pub = self.create_publisher(Bool, "/IndSteer", self.qos) #NIGGA DONT COMMENT THIS OUT LOL
        ##### Services #####
        self.client_log = self.create_client(Trigger, "log_service")

        ##### Timers #####
        self.timer = self.create_timer(0.1, self.timer_callback)


    def joy_callback(self, joy: Joy):
        
        if not self.state:   # The rover is in manual mode  
            
            # Check if the rover is getting joy commands or not, else timeout
            self.joy_last_timestamp = self.get_clock().now()
            # Check all the joy inputs and perform required functions
            if joy.buttons[self.change_dir_button]:
                self.change_dir = not self.change_dir
                self.actual_directiom = "ZedForward" if not self.change_dir else "ArmForward"

            if joy.buttons[self.mode_up_button]:
                sleep(1)
                if self.mode < 4:
                    self.mode += 1

            if joy.buttons[self.mode_down_button]:
                sleep(1)
                if self.mode > 0:
                    self.mode -= 1

            # Check if the GPS logging button is pressed
            if joy.buttons[self.log_btn] == 1 and not self.log_btn_pressed:
                self.log_btn_pressed =True
                self.call_log_service()

            if joy.buttons[self.log_btn] == 0:
                self.log_btn_pressed = False
                            
            if self.steer_islocked and self.full_potential_islocked:
                # This is the case when both the steering and full potential are locked
                # In this case, only changing the wheels orientation is done

                # self.steering_ctrl_locked stores all the buttons input for steering locked condition
                self.steering_ctrl_locked = [
                    joy.buttons[self.forward_btn],    
                    joy.buttons[self.parallel_btn],
                    joy.buttons[self.rotinplace_btn]
                ]
                
                # This variable is to get the fb and lr axes input from the controller
                if self.change_dir:
                    self.drive_ctrl = [-joy.axes[self.fb_axis], joy.axes[self.lr_axis]]
                else: 
                    self.drive_ctrl = [joy.axes[self.fb_axis], joy.axes[self.lr_axis]]
                # There is a fourth axis apparently and this variable stores it
                self.curve_opp_str = joy.axes[4] 

            elif not self.steer_islocked and self.full_potential_islocked:
                # In this case steering is unlocked 

                # self.steering_ctrl_unlocked stores the buttons which are unlocked when steering is unlocked
                self.steering_ctrl_unlocked = [
                    joy.buttons[self.forward_btn],
                    joy.buttons[self.parallel_btn]
                ]
                # self.steering_ctrl_pwm stores the inputs via the joystick axes during steering unlock condition
                self.steering_ctrl_pwm = [
                    joy.axes[self.steer_samedir_axis],
                    joy.axes[self.steer_oppdir_axis]
                ]

            elif self.steer_islocked and not self.full_potential_islocked:
                # In this full potential steering is unlocked

                # self.full_potential_steering holds inputs for all the four wheels, when we want to achieve full potential steering
                self.full_potential_pwm = [
                    joy.axes[self.fl_wheel_axis],
                    joy.axes[self.fr_wheel_axis],
                    joy.axes[self.bl_wheel_axis],
                    joy.axes[self.br_wheel_axis]
                ]
            # Both cannot be unlocked at the same time, hence no case _ required

            # Check the mode changing conditions
            if (joy.axes[self.steer_unlock_axis] == -1.0):  
                # Lock full potential when steering pwm is being toggled
                self.steer_islocked = not self.steer_islocked
                self.full_potential_islocked = True

            elif (joy.axes[self.full_potential_unlock_axis] == -1.0):
                # Lock steering pwm when full potential (individual wheels) control is being toggled
                self.full_potential_islocked = not self.full_potential_islocked
                self.steer_islocked = True

        self.change_dir = False if self.state else self.change_dir
        self.actual_directiom = "ZedForward" if not self.change_dir else "ArmForward" 

        # Changing mode of operation
        if joy.buttons[self.autonomous_btn] == 1: 
            self.state = not self.state


    def gps_callback(self, gps: NavSatFix):
        self.gps_message = gps


    def rotinplace_callback(self, msg: Int8):
        self.rotin = msg.data
        self.get_logger().info(f"self.rotin: {self.rotin}")


    def autonomous_callback(self, msg: Twist):
        if self.state: 
            self.autonomous_vel = msg.linear.x #i removed the constants here 
            self.autonomous_omega = msg.angular.z

            
    def autonomous_control(self):
        # Check first if the rover is in autonomous mode
        if self.state:
            # Just check when to print
            if self.print_ctrl == 0:
                self.get_logger().warn("Rover in autonomous mode. Press A to enter manual mode.")

            ## Wonder what this part does ##
            self.steering_ctrl_locked[0] = 0
            self.steering_ctrl_locked[2] = 0

            if self.rotin == 0:
                self.state_init = [False, False, False]

            elif self.rotin == 1:
                if not self.state_init[0]:
                    self.steering_ctrl_locked[2] = 1
                    self.state_init[0] = True

            elif self.rotin == 2:
                if not self.state_init[1]:
                    self.state_init[0] = False
                    self.steering_ctrl_locked[0] = 1
                    self.state_init[1] = True


    def log_gps(self):
        """
        This function logs the gps to object_coords.csv file.
        The button to activate this on our current controller: BACK button (no. 10 in /joy)
        """
        self.get_logger().info("Preparing to log GPS coords.....")
        if self.gps_message is None:
            self.get_logger().warn("Not getting GPS from topic, GPS message is still None!!!")
            return

        # Open the csv file and write the latitude and longitude from the message
        with open(self.gps_file_name, "a") as gps_log_file:
            writer = csv.writer(gps_log_file)
            latitude = self.gps_message.latitude
            longitude = self.gps_message.longitude
            self.get_logger().info(f"Logging object_number {self.object_number} at GPS ({latitude}, {longitude})!")
            writer.writerow([latitude, longitude])

        # Increment the number of objects so far found
        self.object_number += 1

    def call_log_service(self):
        if not self.client_log.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("log_service not available!")
            return

        req = Trigger.Request()
        future = self.client_log.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Service response success={response.success}, message='{response.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


    def steer(self, initial_angles, final_angles, mode):
        """
        Helper function that steers all the wheels from current angles to target angles according to the specified mode.

        :param initial_angles: current wheel angles in controller order (sequence of floats, radians).
        :param final_angles: target wheel angles in the same order as initial_angles (sequence of floats, radians).
        :param mode: How encoder angles have to be treated as 0 -> relative or 1 -> absolute
        """

        pwm = [0] * 4
        pwm_temp = [0] * 4

        # This condition is to check if the wheels reached the final angle that it was told to go to
        # Within an error threshold

        # Encoder is relative
        relative_angle_conditions = [
            abs(self.enc_data[i] - initial_angles[i]) < abs(final_angles[i]) - self.error_thresh 
            for i in range(4)
        ]

        # Encoder is absolute
        absolute_angle_conditions = [
            abs(self.enc_data[i] - final_angles[i]) > self.error_thresh
            for i in range(4)
        ]

        angle_conditions = [relative_angle_conditions, absolute_angle_conditions]

        if any(angle_conditions[mode]) and (within_time := (time() - self.start_time)):

            # Printing only at certain intervals, to prevent the screen from being filed with data
            if(int(within_time) * 10 % 2 == 0):  
                print("Enc_data:- ", self.enc_data, end = "     ")
                print("Final angles:- ", list(map(add, initial_angles, final_angles)))

            # Finally constraining the PWM
            for i in range(4):
                if (abs(self.enc_data[i]-final_angles[i])>self.error_thresh):
                    pwm_temp[i] = int(self.kp_steer*(final_angles[i]-self.enc_data[i]))
                    if pwm_temp[i]>=0:
                        pwm[i]=min(self.max_steer_pwm, pwm_temp[i])
                    else:
                        pwm[i] = max(-self.max_steer_pwm, pwm_temp[i])
                else:
                    pwm[i] = 0
            
            # PWM message
            self.pwm_msg.data = [
                0,0,0,0,
                pwm[0] * self.init_dir[4],
                pwm[1] * self.init_dir[5],
                pwm[2] * self.init_dir[6],
                pwm[3] * self.init_dir[7]
            ]

            # Publish the PWM message
            self.create_rate(10).sleep()
            self.get_logger().info(f"PWM: {self.pwm_msg.data}")
            self.pwm_pub.publish(self.pwm_msg)

        else:
            # Visual
            print()
            print("***Steering Complete***")
            print()
            self.is_steer = False


    def steering(self):
        """
        Function that decides the final angles based on the conditions of the rover and sends the pwm message
        """
        if self.steer_islocked and self.full_potential_islocked:
            # This is the case when both the steering and full potential are locked
            # In this case, only changing the wheels orientation is done
            self.IndividualSteer = False
            if 1 in self.steering_ctrl_locked:
                # Visualisation
                index = self.steering_ctrl_locked.index(1)
                visualisation_string_dict = {
                    0: "Rotating steering forward",
                    1: "Rotating steering perpendicular to rover",
                    2: "Rotating steering for in place rotation"
                }

                print()
                print(visualisation_string_dict[index])
                print()

                self.steering_complete = False
                self.rotinplace = (self.steering_ctrl_locked[2] == 1)
                self.crab = (self.steering_ctrl_locked[1] == 1)
                self.start_time = time()
                
                final_angles_dict = [
                    [0, 0, 0, 0],
                    [90, 90, 90, 90],
                    [55, -55, 55, -55]
                ]
                if self.steering_ctrl_locked[0] == 1:
                    for i in range(4):
                        self.pwm_msg.data[i+4] = final_angles_dict[0][i]
                elif self.steering_ctrl_locked[1] == 1:
                    for i in range(4):
                        self.pwm_msg.data[i+4] = final_angles_dict[1][i]
                elif self.steering_ctrl_locked[2] == 1:
                    for i in range(4):
                        self.pwm_msg.data[i+4] = final_angles_dict[2][i]
                    print("helo")

                self.final_angles = final_angles_dict[index]
                self.is_steer = True
                # self.steer(initial_angles=[0, 0, 0, 0], final_angles=final_angles, mode=1)
            
            elif (abs(self.curve_opp_str) > 0.2):
                # self.curve_opp_str basically gives a curve motion so all the 
                # wheels kind of align in order to achieve that
                self.steering_complete = False
                self.crab = False
                self.rotinplace = False
                self.start_time = time()

                # Amount of curve needed multiplied with steering multiplier
                temp = int(self.s_arr[self.mode] * self.curve_opp_str / 2.777)
                print(temp)
                # PWM message
                self.pwm_msg.data = [  
                    # Last four values of this message encode the omega for each wheel
                    0,0,0,0,
                    temp * self.init_dir[4],  
                    temp * self.init_dir[5], 
                    -temp * self.init_dir[6], 
                    -temp * self.init_dir[7]
                ]
                sdfsf = [  
                    # Last four values of this message encode the omega for each wheel
                    0,0,0,0,
                    temp * self.init_dir[4],  
                    temp * self.init_dir[5], 
                    -temp * self.init_dir[6], 
                    -temp * self.init_dir[7]
                ]
                print(sdfsf)
                # Visualisation
                print("Encoder angles:-", self.enc_data, end = "       ") 
                print("Mode =", self.mode, end = "      ")
                print("Curving with steering")

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                
    
        elif not self.steer_islocked and self.full_potential_islocked:
            # This is the case when the steering is unlocked

            # Make a deep copy
            self.IndividualSteer = False
            enc_data_new = copy.deepcopy(self.enc_data)

            if 1 in self.steering_ctrl_unlocked:
                index = self.steering_ctrl_unlocked.index(1)

                visualisation_string_dict = {
                    0: "clockwise",
                    1: "anti-clockwise"
                }
                # Visualization
                print()
                print(f"Turning steering {visualisation_string_dict[index]} by 45 deg")
                print()

                self.final_angles = [45 if index == 0 else -45] * 4
                self.steering_complete = False
                self.rotinplace = False
                self.crab = False
                self.start_time = time()
                self.is_steer = True
                # self.steer(initial_angles=enc_data_new, final_angles=final_angles, mode=0)

            # If same dir axis is toggled
            elif (self.steering_ctrl_pwm[0] != 0 and abs(self.steering_ctrl_pwm[1]) < 0.2):     # Edit here to give operator threshold
                self.steering_complete = False
                self.rotinplace = False
                self.crab = False
                self.start_time = time()
                temp = -int(self.s_arr[self.mode] * self.steering_ctrl_pwm[0])
                # Create PWM message
                self.pwm_msg.data = [
                    0,0,0,0,
                    temp * self.init_dir[4],
                    temp * self.init_dir[5],
                    temp * self.init_dir[6],
                    temp * self.init_dir[7]
                ]

                # Visualisation
                print("Encoder angles:-", self.enc_data, end = "       ") 
                print("Mode =", self.mode, end = "      ")
                print("All wheels -> same direction.")

            # If opp dir axis is toggled -> Rotating in place with the velocity from axis
            elif (self.steering_ctrl_pwm[1] != 0 and abs(self.steering_ctrl_pwm[0]) < 0.2):
                # Visualisation
                print()
                print("Moving with rot in place velocities")
                print()

                temp = int(self.max_steer_pwm * self.steering_ctrl_pwm[1])
                self.pwm_msg.data = [
                    0,0,0,0,
                    temp * self.init_dir[4],
                    -temp * self.init_dir[5],
                    -temp * self.init_dir[6],
                    temp * self.init_dir[7]
                ]
                # Visualisation
                print("Rotating in place with velocity =", temp)
                print("Enc_angles:- ", self.enc_data)

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                if (self.print_ctrl == 0):    # Printing only at certain intervals, to prevent the screen from being filed with data
                    print("Steering is unlocked, lock it to perform drive.")

        elif self.steer_islocked and not self.full_potential_islocked:
            self.tuning = 0.5
            # In this case, full_potential_steering is unlocked, so all the wheels can be controlled individually
            # fl_axis toggled -> front left wheel
            self.IndividualSteer = True
            if (self.full_potential_pwm[0] != 0 and abs(self.full_potential_pwm[2]) < 0.2):
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[0] * self.tuning)
                # self.pwm_msg.data = [
                #     0,0,0,0,
                #     temp * self.init_dir[4],
                #     0,0,0
                # ]
                self.pwm_msg.data = [
                    0,0,0,0,0,0,0,
                    temp * self.init_dir[7]
                ]
                # Visualisation
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving front left wheel.")

            # fr_axis toggled -> front right wheel
            elif (self.full_potential_pwm[1] != 0 and abs(self.full_potential_pwm[3]) < 0.2):
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[1] * self.tuning)
                # self.pwm_msg.data = [
                #     0,0,0,0,0,
                #     temp * self.init_dir[5],
                #     0,0
                # ]
                self.pwm_msg.data = [
                    0,0,0,0,0,0,
                    temp * self.init_dir[6],
                    0
                    ]
                # Visualisation
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving front right wheel.")
                
            # bl_axis toggled -> back left wheel
            elif (self.full_potential_pwm[2] != 0 and abs(self.full_potential_pwm[0]) < 0.2): 
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[2] * self.tuning)
                # self.pwm_msg.data = [
                #     0,0,0,0,0,0,
                #     -temp * self.init_dir[6],
                #     0
                #     ]
                self.pwm_msg.data = [
                    0,0,0,0,0,
                    -temp * self.init_dir[5],
                    0,0
                ]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving back left wheel.")

            # br_axis toggled -> back right wheel
            elif (self.full_potential_pwm[3] != 0 and abs(self.full_potential_pwm[1]) < 0.2):    
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[3] * self.tuning)
                # self.pwm_msg.data = [
                #     0,0,0,0,0,0,0,
                #     -temp * self.init_dir[7]
                # ]
                self.pwm_msg.data = [
                    0,0,0,0,
                    -temp * self.init_dir[4],
                    0,0,0
                ]

                # Visualisation
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving back right wheel.")

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                if (self.print_ctrl == 0):    # Printing only at certain intervals, to prevent the screen from being filed with data
                    
                    print("Individual steering control mode unlocked, lock it to perform drive.")

        # At the end, steering is complete
        self.steering_complete = True
        self.start_time = time()


    def drive(self):
        if self.steering_complete and self.steer_islocked and self.full_potential_islocked:
            # Steering is done, and both the types of steering are locked, only then we can drive the rover
            if self.rotinplace: # Wheels are in rotin place alignment
                # Get the velocity by multiplying the drive velocity multiplier with the axis output
                vel = self.autonomous_omega if self.state else self.drive_multipliers[self.mode] * self.drive_ctrl[1] 

                # PWM message
                self.pwm_msg.data = [
                    int(vel) * self.init_dir[0], 
                    -int(vel) * self.init_dir[1],
                    int(vel) * self.init_dir[2],
                    -int(vel) * self.init_dir[3], 
                    55, -55, -55, 55]
                
                if self.print_ctrl == 0:    
                    # Printing only at certain intervals, to prevent the screen from being filled with data   
                    # Print_ctrl is being incremented in main() every time
                    print(f"Rotation speed = {int(vel)}")

            elif self.crab:
                # Get the velocity by multiplying the drive velocity multiplier with the axis output
                vel = self.autonomous_omega if self.state else self.drive_multipliers[self.mode] * self.drive_ctrl[1] 

                # PWM message
                self.pwm_msg.data = [
                    int(vel) * self.init_dir[0], 
                    int(vel) * self.init_dir[1],
                    int(vel) * self.init_dir[2],
                    int(vel) * self.init_dir[3], 
                    90, 90, 90, 90]
                
                if self.print_ctrl == 0:    
                    # Printing only at certain intervals, to prevent the screen from being filled with data   
                    # Print_ctrl is being incremented in main() every time
                    print(f"Crab speed = {int(vel)}") 
            else:  
                # Wheels are not aligned for rotin place
                velocity = self.autonomous_vel if self.state else self.drive_multipliers[self.mode] * self.drive_ctrl[0]
                omega = self.autonomous_omega if self.state else self.drive_multipliers[self.mode] * self.drive_ctrl[1]
        
                avg_velocity = avg_omega = 0

                # Here queue is useful, FIFO right so we keep track of previous
                # three velocities to smoothen the velocity that is actually given to the rover
                if self.vel_prev.full() and self.omega_prev.full():
                    # Calculate the average velocity and omega
                    avg_velocity = sum(self.vel_prev.queue) / self.qsize
                    avg_omega = sum(self.omega_prev.queue) / self.qsize

                    # Remove the first element
                    self.vel_prev.get() 
                    self.omega_prev.get()
                
                # Add new velocity to the queue
                self.vel_prev.put(velocity, True, 2)
                self.omega_prev.put(omega, True, 2)

                print(self.actual_directiom)
                print("Velocity:", (avg_velocity // 0.001) / 1000) # the math part is to only print upto 2 decimals
                print("Omega: ", (avg_omega // 0.001) / 1000)
                print("Mode: ", self.mode)
                print()

                # PWM message
                self.pwm_msg.data[0] = int(avg_velocity + avg_omega) * self.init_dir[0]
                self.pwm_msg.data[1] = int(avg_velocity - avg_omega) * self.init_dir[1]
                self.pwm_msg.data[2] = int(avg_velocity + avg_omega) * self.init_dir[2]
                self.pwm_msg.data[3] = int(avg_velocity - avg_omega) * self.init_dir[3]
                 
            self.pwm_pub.publish(self.pwm_msg)
    

    def timer_callback(self):
        # Publish the state, required for autonomous mission
        state_msg = Bool()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        indsteer_msg = Bool()
        indsteer_msg.data = self.IndividualSteer
        self.IndSteer_pub.publish(indsteer_msg)

        if not self.state:
            now = self.get_clock().now()
            time_since_last_joy_msg = (now - self.joy_last_timestamp).nanoseconds / 1e9
            self.get_logger().info(f"Time diff: {time_since_last_joy_msg}")
            self.get_logger().info(f"{time_since_last_joy_msg >= self.timeout_time}")
            if time_since_last_joy_msg >= self.timeout_time:
                self.get_logger().warn("No joy commands have been recieved for a long time, locking rover...")
                # Cutting the controls of joy
                self.state = True       

        if self.in_steer:
            self.get_logger().info("Preparing to steer...")

            if not self.steer_islocked:
                # This is relative
                self.steer(initial_angles=[0, 0, 0, 0], final_angles=self.final_angles, mode=0)
            else:
                # This is absolute
                self.steer(initial_angles=self.enc_data, final_angles=self.final_angles, mode=1)

        else:
            self.get_logger().info(f"{self.steering_ctrl_locked}")

            if self.rotin != 0:
                self.get_logger().warn(f"{self.rotin}")

            if self.state:
                self.get_logger().info(f"auto vel is {self.autonomous_vel}")
                self.get_logger().info(f"auto omega is {self.autonomous_omega}")

            self.autonomous_control()
            self.steering()
            self.drive()
            self.get_logger().info(f"PWM: {self.pwm_msg.data}")
            self.pwm_pub.publish(self.pwm_msg)
            self.print_ctrl = (self.print_ctrl + 1) % self.prints_per_iter


def main(args=None):
    r.init(args=args)
    node = Drive()
    r.spin(node)
    r.shutdown()


if __name__ == "__main__":
    main()
    





    # History Lesson
    # If the forward button is pressed -> Align all the wheels forward
    # if self.steering_ctrl_locked[0] == 1:
    #     # Visualisation
    #     print()
    #     print("Rotating steering forward")
    #     print()

    #     self.steering_complete = False
    #     self.rotinplace = False
    #     self.start_time = time()
    #     self.steer([0,0,0,0],[0,0,0,0],1)

    # # Parallel button is pressed -> Align all the wheels perpendicular to the rover
    # elif self.steering_ctrl_locked[1] == 1:
    #     # Visualisation
    #     print()
    #     print("Rotating steering perpendicular to rover")
    #     print()

    #     self.steering_complete = False
    #     self.rotinplace = False
    #     self.start_time = time()
    #     self.steer([0,0,0,0],[90,90,90,90],1)

    # # Rotin button is pressed -> Align wheels for rotating in place
    # elif(self.steering_ctrl_locked[2] == 1):
    #     # Visualisation
    #     print()
    #     print("Rotating steering for in place rotation")
    #     print()
    #     # rotinplace must be made true only in manual mode
    #     # Then why isnt there a condition that takes self.autonomous_btn into account?
    #     self.rotinplace = True
    #     self.steering_complete = False
    #     self.start_time = time()
    #     self.steer([0,0,0,0],[55,-55,-55,55],1)


                    
    # # Forward button is pressed -> Turn all wheels 45 deg from their current state in clockwise direction
    # if (self.steering_ctrl_unlocked[0] == 1):
    #     # Visualisation
    #     print()
    #     print("Turning steering clockwise by 45 deg")
    #     print()

    #     self.steering_complete = False
    #     self.rotinplace = False
    #     self.start_time = time()
    #     self.steer(enc_data_new, [45,45,45,45], 0) # Initial angle, final angle, mode=0 for relative

    # # Parallel button pressed -> Turn all wheels by 45 deg anticlockwise
    # elif (self.steering_ctrl_unlocked[1] == 1):
    #     # Visualisation
    #     print()
    #     print("Turning steering anti-clockwise by 45 deg")
    #     print()

    #     self.steering_complete = False
    #     self.rotinplace = False
    #     self.start_time = time()
    #     self.steer(enc_data_new, [-45,-45,-45,-45], 0) # initial angle, final angle, mode=0 for relative

    #self.d_arr = [25,35,50,75,110]                     # Same as galileo drive multipliers


