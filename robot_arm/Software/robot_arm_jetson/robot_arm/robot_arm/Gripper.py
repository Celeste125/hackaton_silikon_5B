import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

from std_msgs.msg import String

from robot_arm_interfaces.action import Gripper 

class GripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server', namespace='Team2/robot_arm')
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.cmd_pub = self.create_publisher(
            String, 
            'robot_serial_out', 
            10
        )
        
        self.feedback_sub = self.create_subscription(
            String, 
            'gripper/serial', 
            self.serial_rx_callback, 
            10,
            callback_group=self.cb_group
        )
        
        self.latest_serial_data = ""
        self.goal_reached = False

        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper',
            self.execute_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info("Gripper Action Server is ready (using Topic routing).")

    def serial_rx_callback(self, msg):
        """Wird aufgerufen, sobald die Serial-Node etwas empfängt und publisht."""
        incoming_data = msg.data.strip()
        
        if incoming_data.startswith("END:"):
            self.goal_reached = True
            self.latest_serial_data = incoming_data
        else:
            self.latest_serial_data = incoming_data

    def execute_callback(self, goal_handle):
        self.goal_reached = False
        self.latest_serial_data = ""
        last_published_feedback = ""
        
        # Welcher Befehl soll gesendet werden?
        if goal_handle.request.open:
            self.get_logger().info('Received goal: open')
            command_str = "OPEN_GRIPPER"
        else:
            self.get_logger().info('Received goal: closed')
            command_str = "CLOSE_GRIPPER"
        
        # 1. Befehl via ROS Topic an die Serial-Node senden
        cmd_msg = String()
        cmd_msg.data = command_str
        self.cmd_pub.publish(cmd_msg)
        
        # Prepare the feedback message object
        feedback_msg = Gripper.Feedback()
        
        # 2. Wait for feedback and completion
        while not self.goal_reached:
            # Handle cancellation requests from the client
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled by client.')
                
                # Sende STOP Befehl über Topic
                stop_msg = String()
                stop_msg.data = "STOP"
                self.cmd_pub.publish(stop_msg)
                
                return Gripper.Result(success=False, message="Action canceled.")

            # Veröffentliche Feedback, wenn es neue Daten gibt
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
                            
                            if key == "POSITION":
                                # Action file expects int32, but incoming string might be float
                                feedback_msg.position = int(float(val))
                            elif key == "CURRENT":
                                feedback_msg.current = int(float(val))
                            elif key == "MESSAGE":
                                feedback_msg.message = val
                                
                        goal_handle.publish_feedback(feedback_msg)
                        last_published_feedback = self.latest_serial_data
                    
                except ValueError as e:
                    self.get_logger().warn(f"Could not parse serial data: {self.latest_serial_data}. Error: {e}")
            
            # Small sleep to prevent the while loop from consuming 100% CPU
            # In dieser Pause verarbeitet der Executor neue Subscriber-Nachrichten
            time.sleep(0.01)

        self.get_logger().info("Robot reported it reached the position.")

        # 3. Mark the goal as successful and return the result
        goal_handle.succeed()
        
        payload = self.latest_serial_data[4:]
        parts = payload.split(';')
        result = Gripper.Result()
        for part in parts:
                            # Use maxsplit=1 in case the message itself contains colons
                            key, val = part.split(':', 1)
                            
                            if key == "SUCCESS":
                                if val == "TRUE":
                                     result.success = True
                                else:
                                     result.success = False
                                
                            elif key == "GRIPPED":
                                if val == "TRUE":
                                     result.gripped = True
                                else:
                                     result.gripped = False
                            elif key == "MESSAGE":
                                result.message = val
          
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = GripperActionServer()
    
    # Nutze den MultiThreadedExecutor anstatt der Standard-Spin-Methode
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()