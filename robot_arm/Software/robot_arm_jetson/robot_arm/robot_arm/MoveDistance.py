import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

from std_msgs.msg import String, Int32

# Import the custom action definition we created above
# Replace 'my_robot_interfaces' with your actual package name
from robot_arm_interfaces.action import MoveDistance

class SerialMoveActionServer(Node):
    def __init__(self):
        super().__init__('serial_move_action_server', namespace='Team2/robot_arm')

        self.cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(
            String, 
            'robot_serial_out', 
            10
        )

        self.feedback_sub = self.create_subscription(
            String, 
            'arm/serial', 
            self.serial_rx_callback, 
            10,
            callback_group=self.cb_group
        )

        self.latest_auto_distance = None
        self.distance_sub = self.create_subscription(
            Int32,
            'ultrasound/distance',
            self.distance_callback,
            10,
            callback_group=self.cb_group
        )

        self.latest_serial_data = ""
        self.goal_reached = False

        self._action_server = ActionServer(
            self,
            MoveDistance,
            'move_robot',
            self.execute_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info("Move Action Server is ready (using Topic routing).")



    def distance_callback(self, msg):
        """Stores the latest distance received from /Team2/distance."""
        self.latest_auto_distance = msg.data

    def serial_rx_callback(self, msg):
        """Wird aufgerufen, sobald die Serial-Node etwas empfängt und publisht."""
        incoming_data = msg.data.strip()
        
        if incoming_data.startswith("END:"):
            self.goal_reached = True
            self.latest_serial_data = incoming_data
        else:
            # Speichere die Daten für das Action-Feedback
            self.latest_serial_data = incoming_data

    def execute_callback(self, goal_handle):
        # Setze Status-Variablen für neuen Goal-Aufruf zurück
        self.goal_reached = False
        self.latest_serial_data = ""
        last_published_feedback = ""

        if goal_handle.request.mode == "DISTANCE":
            self.get_logger().info('Distance mode.')
            command_str = "MOVE_ARM:" + str(goal_handle.request.distance)
        elif goal_handle.request.mode == "STACK":
            self.get_logger().info('Stacking mode.')
            command_str = "STACK_MOVE_ARM:" + str(goal_handle.request.distance)
        elif goal_handle.request.mode == "PRESET":
            if goal_handle.request.preset != "NONE":
                command_str = "PRE_MOVE_ARM:" + str(goal_handle.request.preset)
            else:
               self.get_logger().warn('No preset set!') 
               return MoveDistance.Result(success=False, message="Action canceled.")
        elif goal_handle.request.mode == "AUTO":
            measured_distance = self.latest_auto_distance
            validated_distance = goal_handle.request.distance
            if validated_distance >= 0:
                self.get_logger().info(
                    f'Auto mode. Using validated camera-dock distance: '
                    f'{validated_distance} mm; ultrasonic diagnostic: '
                    f'{measured_distance} mm'
                )
                command_str = "MOVE_ARM:" + str(validated_distance)
            elif (
                measured_distance is not None
                and 20 <= measured_distance <= 250
            ):
                self.get_logger().info(
                    'Auto mode has no validated distance; using plausible '
                    f'ultrasonic fallback: {measured_distance} mm'
                )
                command_str = "MOVE_ARM:" + str(measured_distance + 10)
            else:
                self.get_logger().warn(
                    'AUTO mode has neither a validated camera-dock distance '
                    'nor a plausible ultrasonic fallback.'
                )
                return MoveDistance.Result(
                    success=False,
                    message="Action canceled. No valid auto distance.",
                )

        # 1. Befehl via ROS Topic an die Serial-Node senden
        cmd_msg = String()
        cmd_msg.data = command_str
        self.cmd_pub.publish(cmd_msg)
        
        # Prepare the feedback message object
        feedback_msg = MoveDistance.Feedback()

        while not self.goal_reached:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled by client.')
                
                # Sende STOP Befehl über Topic
                stop_msg = String()
                stop_msg.data = "STOP"
                self.cmd_pub.publish(stop_msg)
                
                return MoveDistance.Result(success=False, message="Action canceled.")
            if self.latest_serial_data and self.latest_serial_data != last_published_feedback:
                try:
                    # Check for the new format prefix
                    if self.latest_serial_data.startswith("FEEDBACK:"):
                        # Remove the "FEEDBACK:" prefix (9 characters) before parsing
                        payload = self.latest_serial_data[9:]
                        
                        # Parse the remaining string: "POSITION:22.3;CURRENT:10.52;MESSAGE:This is a test message"
                        parts = payload.split(';')
                        
                        for part in parts:
                            # Use maxsplit=1 in case the message itself contains colons
                            key, val = part.split(':', 1)
                            
                            if key == "POSITION_1":
                                # Action file expects int32, but incoming string might be float
                                feedback_msg.angle_1 = int(float(val))
                            elif key == "POSITION_2":
                                feedback_msg.angle_2 = int(float(val))
                            elif key == "MESSAGE":
                                feedback_msg.message = val
                                
                        goal_handle.publish_feedback(feedback_msg)
                        last_published_feedback = self.latest_serial_data
                    
                except ValueError as e:
                    self.get_logger().warn(f"Could not parse serial data: {self.latest_serial_data}. Error: {e}")
            time.sleep(0.01)

        self.get_logger().info("Robot reported it reached the position.")

        # 3. Mark the goal as successful and return the result
        goal_handle.succeed()
        
        payload = self.latest_serial_data[4:]
        parts = payload.split(';')
        result = MoveDistance.Result()
        for part in parts:
            # Use maxsplit=1 in case the message itself contains colons
            key, val = part.split(':', 1)
                            
            if key == "SUCCESS":
                if val == "TRUE":
                    result.success = True
                else:
                    result.success = False
                                
            elif key == "REACHABLE":
                if val == "TRUE":
                    result.reachable = True
                else:
                    result.reachable = False
            elif key == "MESSAGE":
                result.message = val
          
        return result
            

'''
class SerialMoveActionServer(Node):
    def __init__(self):
        super().__init__('serial_move_action_server')
        
        # Configure the Serial connection
        # Update the port (e.g., /dev/ttyUSB0 or COM3) and baudrate to match your robot
        self.serial_port = '/dev/ttyACM0'
        self.baudrate = 115200
        
        try:
            # timeout=0.1 ensures the readline() doesn't block indefinitely
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to robot on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise SystemExit

        # Initialize the Action Server
        self._action_server = ActionServer(
            self,
            MoveDistance,
            'move_robot',
            self.execute_callback
        )
        self.get_logger().info("Move Action Server is ready.")

    def execute_callback(self, goal_handle):
        target_distance = goal_handle.request.distance
        self.get_logger().info(f'Received goal: {target_distance}')

        # 1. Send the command to the robot via serial
        # Assuming the robot expects a command like "MOVE 10.5\n"
        command = f"DISTANCE {target_distance}\n"
        self.ser.write(command.encode('utf-8'))
        
        # Prepare the feedback message object
        feedback_msg = MoveDistance.Feedback()
        
        # 2. Wait for feedback and completion
        while True:
            # Handle cancellation requests from the client
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled by client.')
                
                # Optionally send a stop command to the robot here
                # self.ser.write(b"STOP\n")
                
                return MoveDistance.Result(success=False, message="Action canceled.")

            # Check if there is data waiting in the serial buffer
            if self.ser.in_waiting > 0:
                # Read the line and decode it
                incoming_data = self.ser.readline().decode('utf-8').strip()
                
                # Check for the completion signal from the robot
                if incoming_data == "REACHED":
                    self.get_logger().info("Robot reported it reached the position.")
                    break
                
                # Otherwise, it's continuous feedback (e.g., current distance or encoder ticks)
                if incoming_data:
                    feedback_msg.current_status = incoming_data
                    goal_handle.publish_feedback(feedback_msg)
            
            # Small sleep to prevent the while loop from consuming 100% CPU
            time.sleep(0.01)

        # 3. Mark the goal as successful and return the result
        goal_handle.succeed()
        
        result = MoveDistance.Result()
        result.success = True
        result.message = "Successfully moved the requested distance."
        
        return result
'''

def main(args=None):
    rclpy.init(args=args)
    action_server = SerialMoveActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
