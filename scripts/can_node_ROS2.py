import rclpy
from rclpy.node import Node
import can
import threading
import time
import struct
import signal 
import sys
from std_msgs.msg import Int8, Float32, Int32MultiArray, Int32MultiArray, Float32MultiArray

class CAN_Class(Node):
    def __init__(self):
        super().__init__("can_node")
        self.get_logger().info("Calling super() init method")
        self.get_logger().info("Creating subscribers")
        self.motor_pwm_sub = self.create_subscription(Int32MultiArray, "/motor_pwm", self.joy_callback, 10)        
        self.test_can_sub = self.create_subscription(Int8, "/test_can", self.int8callback, 10)

        self.final_message_pub = self.create_publisher(Float32, "/final_message", 10)
        self.enc_pub = self.create_publisher(Float32MultiArray, "/enc_auto", 10)

        self.enc_msg = Float32MultiArray()

        self.msg = can.Message(
            arbitration_id=0x0B1,
            is_extended_id=False,
            dlc=7,  
            data= [1,2,3,4,5,6,0]
        )

        self.decoded_encoder_value = 0.0
        ### Third motor's count decreases as it rotates clockwise, that's why the enc_factor is -ve
        self.enc_factor = [90/268,90/268,-90/268,90/268]	
        self.all_msgs = []
        self.bus = can.ThreadSafeBus(channel='can0', bustype='socketcan', fd=True)
	    ### Lock doesn't allow other threads to access a shared resource at the same time
        self.data_lock = threading.Lock()
	    ### Event tells the send_messages thread when there's new data ready to be sent.	
        self.event = threading.Event()
        receiver_thread = threading.Thread(target=self.receive_messages, args=())
        ### deamon makes the thread a one which runs in the background without interrupting the program
        receiver_thread.daemon = True
        receiver_thread.start()

        self.get_logger().info("I am about to send")
        self.total_start_time = time.time()
        sender_thread = threading.Thread(target=self.send_messages, args=())
        sender_thread.daemon = True
        sender_thread.start()
        self.drive_dir = [1,1,1,1,-1,-1,-1,-1]

        self.get_logger().info("I am about to publish")
        publisher_thread = threading.Thread(target=self.publish_messages, args=())
        publisher_thread.daemon = True
        publisher_thread.start()
        self.enc_data=[0.0,0.0,0.0,0.0,0.0,0.0]

        ### BMS Part
        self.bms_can_msg_ids = [0x37, 0x38, 0x39]
        self.bms_pub = self.create_publisher(Float32MultiArray, '/bms/battery_values', 10)
        self.cell_voltages = [0.0]*12
        self.get_logger().info("Created BMS Publisher")
        self.received_bms = set()
        self.bms_debug=False
    
    def int8callback(self, msg: Int8):
        self.get_logger().info("In Callback")
        #self.data_lock.acquire()
        #if(self.data_lock.locked() == True):
        ### Acquiring the threading lock, so the other callback doesn't use the can message at the same time
        with self.data_lock:
            self.msg.arbitration_id = 0x0B1
            self.msg.data = bytearray([msg.data])
            ### Setting the data length to 1 byte
            self.msg.dlc = 1
            ### Says it's ready to be sent
            self.event.set()
            #self.data_lock.release()


    def joy_callback(self, msg: Int32MultiArray):
        #self.data_lock.acquire()
        #if(self.data_lock.locked() == False):
            with self.data_lock:
                self.msg.arbitration_id = 0x001
                ###Multiplies the joystick input value by a direction factor,divides the result by 2 and offsets it to ensure the finalvalue is within the allowed 0–255 range
                self.msg.data =  bytearray([(self.drive_dir[i] * msg.data[i]) // 2 + 127 for i in range(min(8, len(msg.data)))])
                self.msg.dlc = len(self.msg.data)
                self.event.set()
            # self.data_lock.release()
            
            
    def publish_messages(self):
        while True:
           try:
               #self.get_logger().info("----------------------------------------------")
               self.final_message_pub.publish(Float32(data=self.decoded_encoder_value))
               #self.get_logger().info(f"I published the message {self.decoded_encoder_value}")
               time.sleep(0.1)
           except KeyboardInterrupt:
               self.get_logger().info("YOU DARE KeyboardInterrupt ME!!!")
               break
           

    def receive_messages(self):
        while True:
            try:
                msg = self.bus.recv()

                if msg is None:
                    raise TypeError("msg is apparently None")
                
                self.get_logger().info(f"Found id: {msg.arbitration_id}")
                self.get_logger().warn(f"msg={msg}")
                data = msg.data
                # Steering motor part
                    #self.enc_msg.data=[0,0,0,0]
                
                ### BMS Part
                if msg.arbitration_id in self.bms_can_msg_ids:
                    try:
                    	### Getting index values as 0,1,2,3
                        frame_index = msg.arbitration_id - 0x37
                        ### One can message contains values of 4 cells
                        base = frame_index * 4
                        
                        for j in range(4):
                            ### Extracts two bytes data for current cell voltage	
                            raw_bytes = msg.data[j*2:j*2+2]
                            ### Converts the hexadecimal number, whose format is different to decimal number( Eg. b'\x10\x27' is raw bytes (0x2710) and scaled will be 10000)
                            scaled, = struct.unpack('<H', raw_bytes)
                            voltage = scaled / 10000.0
                            self.cell_voltages[base + j] = voltage

                        self.received_bms.add(frame_index)
                        if len(self.received_bms) == 3:
                            arr_msg = Float32MultiArray()
                            arr_msg.data = self.cell_voltages
                            self.bms_pub.publish(arr_msg)

                            self.get_logger().info("BMS Data published")
                            self.received_bms.clear()

                    except Exception as e:
                        self.get_logger().error(f"BMS: Error processing message: {e}")                    
                    
                    continue
                
                ### For encoder values (The index values are dipswitch values of sterring motor drivers)
                if index in [0x1b, 0x1c, 0x1d, 0x1e]:
                    ### Extracts the first two bytes of data and decodes them as a signed integer using little-endian format.
                    self.decoded_encoder_value = int.from_bytes(data[0:2], "little", signed=True)
                    ### Getting index values as 0,1,2,3
                    index = msg.arbitration_id - 0x1b
                    ### Multiply the encoder value to the enc_factor to get the angle to rotate for the encoder
                    self.enc_data[index] = self.enc_factor[index] * self.decoded_encoder_value
                    self.enc_msg.data = self.enc_data
                
                    #self.decoded_encoder_value = int.from_bytes(data[0:1],"big", signed=True) 
                    self.all_msgs.append(self.decoded_encoder_value)
                    self.get_logger().info(f"Message Received = {self.decoded_encoder_value}")

                    if msg:
                        self.get_logger().info(f"Received message: {msg}")
                        self.get_logger().info(f"Recieved value is {self.decoded_encoder_value}")
                        self.enc_pub.publish(self.enc_msg)

            except can.CanError as e:
                self.get_logger().info(f"Error receiving message: {e}")


    def send_messages(self):
        while True:
            try: 
                #event_set = self.event.wait() 
                time.sleep(0.05)
                with self.data_lock:
                    self.bus.send(self.msg)
                    #self.event.clear()
                    self.get_logger().info("Message sent successfully:")
                    #self.get_logger().info(self.msg.data)
                    msg_data = bytearray(self.msg.data)
                    self.get_logger().info(f"msg_data {msg_data}\n")
                    #self.get_logger().info("Data sent is:")
                    #for i in range(len(self.msg.data)):
                        #print("msg_data ", msg_data[i])
                        #self.get_logger().info(int(msg_data[i]))
            except can.CanError as e:
                self.get_logger().info(f"Error sending message: {e}")


                
def signal_handler(sig, frame):

    print("**********************Keyboard interrupt************************")
    sys.exit(0)


signal.signal(signal.SIGINT,signal_handler)


def main():
    rclpy.init()
    node = CAN_Class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received.")
    finally:
        print("Shutting down...")
        try:
            node.bus.shutdown()  # Or node.bus.close(), depending on CAN lib
        except Exception as e:
            print(f"Error during bus shutdown: {e}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



# def mainthread(self):

#     rate = rospy.Rate(10)
#     try:
#         while not rospy.is_shutdown():
#             rospy.spin()
#     #except KeyboardInterrupt:
#     #    self.get_logger().info("Interrupted by user")



#     #    sys.exit(0)
#     finally:
#         sys.exit(0)
#         self.bus.shutdown()
#         self.get_logger().info("In finally block")
