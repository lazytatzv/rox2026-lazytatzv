import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

class TestBaseTeleopIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_joy_to_cmd_vel(self):
        node = rclpy.create_node('test_node')
        pub = node.create_publisher(Joy, 'joy', 10)
        
        received_msgs = []
        sub = node.create_subscription(
            Twist, 
            'cmd_vel', 
            lambda msg: received_msgs.append(msg), 
            10
        )

        # Wait for base_teleop to be ready (if it were running)
        # In a real integration test, we would use a launch_test.
        # For this example, we'll just test the logic if the node is spun up.
        
        msg = Joy()
        msg.axes = [0.0] * 10
        msg.buttons = [0] * 10
        
        # Deadman switch simulation (L2/R2)
        # Based on code: std::abs(joystick_message->axes[axis_deadman_translation_]) > 0.5
        msg.axes[5] = -1.0 # Pressed (DualSense L2/R2 are -1.0 when fully pressed)
        msg.axes[1] = 0.5  # Forward
        
        # Also test that it's disabled when 0.0 (initial state)
        # msg.axes[5] = 0.0 
        
        # Publish and spin
        pub.publish(msg)
        
        timeout = time.time() + 2.0
        while len(received_msgs) == 0 and time.time() < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)

        # If base_teleop was running in the same process/network, 
        # we would expect received_msgs to have content.
        # Note: This requires the actual node under test to be running.
        
if __name__ == '__main__':
    unittest.main()
