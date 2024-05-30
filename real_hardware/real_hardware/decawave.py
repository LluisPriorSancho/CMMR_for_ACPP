import rclpy
from rclpy.node import Node
from custom_interfaces.msg import DecawavePosition, DecawaveList
import serial
import time
import math

class DecawaveBridgeNode(Node):
    def __init__(self):
        super().__init__("decawave_bridge_node")
        self.get_logger().info("Starting DecawaveBridgeNode.")

        # Argument to turn on/off the decawave receiver
        first_start = self.declare_parameter(
          "first_start", False).get_parameter_value()
        
        # Create the topic to publish the data
        self.decawave_data_pub = self.create_publisher(DecawaveList, "decawave_data", 10)
        self.data_list = DecawaveList()
        self.list = []

        # Open serial communication
        self.DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
        self.get_logger().info("Connected to " + self.DWM.name)
        self.DWM.reset_input_buffer()
        self.DWM.reset_output_buffer()
        self.DWM.write("\r\r".encode())
        time.sleep(1)
        
        # Turn on or off the receiver
        if first_start.bool_value:
            self.get_logger().info("Sending initial command: lep")
            self.DWM.write("lep\r".encode())
            time.sleep(1)
        else:
            self.get_logger().info("System already prepared")
        self.DWM.reset_input_buffer()
        self.DWM.reset_output_buffer()
        self.publish_decawave_data()

    def decawave_shutdown_sequence(self):
        self.get_logger().info("Shutting down")
        self.DWM.write("\r".encode())
        self.DWM.close()

    def publish_decawave_data(self):
        while rclpy.ok():
            try:
                #self.DWM.reset_input_buffer()
                #self.DWM.reset_output_buffer()
                # When a message is received:
                line = self.DWM.readline()
                if line:
                    # If the message is valid:
                    #self.get_logger().info("MSG:   " + line.decode("utf-8"))
                    if len(line) >= 20:
                        
                        # Get the position, name of the tag and prepare to publish a topic
                        parse = line.decode().split(",")
                        dwm_name = parse[2]
                        x_pos = parse[3]
                        y_pos = parse[4]
                        z_pos = parse[5]
                        tag_data = DecawavePosition()
                        tag_data.name = dwm_name
                        tag_data.time = self.get_clock().now().to_msg()
                        tag_data.x = float(x_pos)
                        tag_data.y = float(y_pos)
                        tag_data.z = float(z_pos)
                        
                        # Check that the coordinates are valid
                        if math.isnan(tag_data.x) or math.isnan(tag_data.y) or math.isnan(tag_data.z):
                            continue
                        tag_name_exists = False
                        exist_index = None
                        
                        # Check if the tag already exist, if not, add it
                        for i in range(len(self.list)):
                            if self.list[i].name == dwm_name:
                                tag_name_exists = True
                                exist_index = i
                        if tag_name_exists:
                            self.list[exist_index] = tag_data
                        else:
                            self.list.append(tag_data)
                            
                        # Send the topic with the data actualized
                        self.data_list.amount = len(self.list)
                        self.data_list.time = self.get_clock().now().to_msg()
                        self.data_list.decawaveposition = self.list
                        self.decawave_data_pub.publish(self.data_list)

            except Exception as ex:
                print(ex)
                break
            time.sleep(0.0005)

def main(args=None):
    rclpy.init(args=args)
    try:
        decawave_bridge_node = DecawaveBridgeNode()
        try: 
            rclpy.spin(decawave_bridge_node)
        finally:
            decawave_bridge_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
