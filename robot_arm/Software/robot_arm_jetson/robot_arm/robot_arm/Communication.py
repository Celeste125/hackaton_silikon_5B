import rclpy
from rclpy.node import Node
import serial

# Import standard message types
from std_msgs.msg import Float32, Int32, String

class Communication(Node):
    def __init__(self):
        super().__init__('robot_communication', namespace='Team2/robot_arm')
        
        # 1. Configuration (In production, use ROS 2 Parameters for these)
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.serial_buffer = ""
        
        # 2. Setup the Publishers
        self.distance_pub = self.create_publisher(Int32, 'ultrasound/distance', 10)
        self.distance_arm_pub = self.create_publisher(Int32, 'ultrasound/arm', 10)

        self.arm_serial_pub = self.create_publisher(String, 'arm/serial', 10)
        self.arm_angle1_pub = self.create_publisher(Float32, 'arm/angle_1', 10)
        self.arm_angle2_pub = self.create_publisher(Float32, 'arm/angle_2', 10)

        self.gripper_serial_pub = self.create_publisher(String, 'gripper/serial', 10)
        self.gripper_state_pub = self.create_publisher(Int32, 'gripper/state', 10)

        self.command_sub = self.create_subscription(
            String, 
            'robot_serial_out', 
            self.write_serial_data, 
            10
        )
        
        # 3. Initialize the Serial Connection
        try:
            # timeout=0 makes it non-blocking. 
            # We only read when data is verified to be in the buffer.
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0)
            self.get_logger().info(f"Successfully connected to {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.port}: {e}")
            # Exit if hardware isn't found
            raise SystemExit 

        # 4. Create a Timer to poll the serial port (e.g., 100Hz)
        # This keeps ROS spinning smoothly without locking the thread.
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.read_serial_data)
    
    def write_serial_data(self, msg):
        """Triggered whenever a message is published to /robot/commands"""
        if self.ser.is_open:
            try:
                # 1. Extract the string from the ROS message
                command_str = "C." + msg.data
                
                # 2. Add a newline character if your microcontroller expects it
                command_str += "\n"
                
                # 3. Convert the Python string to bytes and write to the port
                self.ser.write(command_str.encode('utf-8'))
                
                self.get_logger().debug(f"Sent to hardware: {command_str.strip()}")
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial: {e}")

    def read_serial_data(self):
        """Called repeatedly by the timer to check for and parse data."""
        if self.ser.in_waiting > 0:
            try:
                # Read all currently available bytes (instead of readline)
                new_data = self.ser.read(self.ser.in_waiting).decode('utf-8')
                self.serial_buffer += new_data
                
                # Check if we have received a complete line (indicated by newline)
                if '\n' in self.serial_buffer:
                    # Split the buffer by newlines
                    lines = self.serial_buffer.split('\n')
                    
                    # The last element is either an empty string (if it ended perfectly with \n)
                    # or an incomplete line. Keep it in the buffer for the next timer cycle.
                    self.serial_buffer = lines[-1]
                    
                    # Process all complete lines
                    for line in lines[:-1]:
                        clean_line = line.strip()
                        if clean_line:
                            self.get_logger().info(clean_line) # Only printed once now
                            self.parse_and_publish(clean_line)
                            
            except UnicodeDecodeError:
                self.get_logger().warning("Received malformed bytes (not UTF-8).")
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

    def parse_and_publish(self, line):
        """Routes the parsed string to the correct ROS topic."""
        try:
            if line.startswith("A.STATUS:"):
                
                data_str = line[len("A.STATUS:"):]
                parts = data_str.split(";")
                
                for part in parts:
                    if ":" not in part:
                        continue
                        
                    key, value = part.split(":")
                    
                    if key == "DISTANCE_1":
                        msg = Int32()
                        msg.data = int(value)
                        self.distance_pub.publish(msg)

                    elif key == "DISTANCE_2":
                        msg = Int32()
                        msg.data = int(value)
                        self.distance_arm_pub.publish(msg)
                        
                    elif key == "ANGLE1":
                        msg = Float32()
                        msg.data = float(value)
                        self.arm_angle1_pub.publish(msg)
                        
                    elif key == "ANGLE2":
                        msg = Float32()
                        msg.data = float(value)
                        self.arm_angle2_pub.publish(msg)
                        
                    elif key == "GRIPPERSTATE":
                        msg = Int32()
                        msg.data = int(value)
                        self.gripper_state_pub.publish(msg)
            # Example 1: Temperature Sensor (Prefix "TEMP:")
            if line.startswith("A.SCAN:"):

                data_str = line[len("A.SCAN:"):]
                parts = data_str.split(";")
                                
                for part in parts:
                    if ":" not in part:
                        continue
                                        
                    key, value = part.split(":")
                                    
                    if key == "DISTANCE_1":
                        msg = Int32()
                        msg.data = int(value)
                        self.distance_pub.publish(msg)
                
                    elif key == "DISTANCE_2":
                        msg = Int32()
                        msg.data = int(value)
                        self.distance_arm_pub.publish(msg)
    
                
            # Example 2: Motor Encoder (Prefix "ENC:")
            elif line.startswith("A.ARM."):
                value_str = line[len("A.ARM."):]
                
                msg = String()
                msg.data = value_str
                self.arm_serial_pub.publish(msg)

            elif line.startswith("A.GRIPPER."):
                value_str = line[len("A.GRIPPER."):]
                
                msg = String()
                msg.data = value_str
                self.gripper_serial_pub.publish(msg)
                
            else:
                self.get_logger().debug(f"Ignored unknown serial string: {line}")
                
        except ValueError:
            self.get_logger().warning(f"Failed to cast value to number: {line}")
        except IndexError:
            self.get_logger().warning(f"Malformed string format (missing colon): {line}")

    def destroy_node(self):
        """Ensure the serial port is released back to the OS on shutdown."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Communication()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()