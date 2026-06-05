#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import time
import math

class MotorTestTool(Node):
    def __init__(self):
        super().__init__('motor_test_tool')
        self.pub = self.create_publisher(Float64, '/test_motor/desired_velocity', 10)
        self.running_sequence = False
        self.stop_event = threading.Event()

    def send_speed(self, speed):
        msg = Float64()
        msg.data = float(speed)
        self.pub.publish(msg)

    def run_step_test(self, speed, duration):
        self.get_logger().info(f'Starting step test: speed={speed}, duration={duration}s')
        self.send_speed(speed)
        time.sleep(duration)
        self.send_speed(0.0)
        self.get_logger().info('Step test finished.')

    def run_sine_test(self, amplitude, frequency, duration):
        self.get_logger().info(f'Starting sine test: amp={amplitude}, freq={frequency}Hz, duration={duration}s')
        start_time = time.time()
        while time.time() - start_time < duration and not self.stop_event.is_set():
            t = time.time() - start_time
            val = amplitude * math.sin(2 * math.pi * frequency * t)
            self.send_speed(val)
            time.sleep(0.02) # 50Hz
        self.send_speed(0.0)
        self.get_logger().info('Sine test finished.')

def main():
    rclpy.init()
    node = MotorTestTool()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n=== ROX2026 Motor Test Tool ===")
    print("Target: /test_motor/desired_velocity")
    
    try:
        while True:
            print("\nCommands:")
            print("  [value] : Send constant speed (e.g., 0.5)")
            print("  step    : Run step test (prompt for speed/duration)")
            print("  sine    : Run sine wave test (prompt for amp/freq/duration)")
            print("  s       : Emergency Stop (0.0)")
            print("  q       : Quit")
            
            cmd = input("\n>> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 's':
                node.send_speed(0.0)
                print("Stopped.")
            elif cmd == 'step':
                try:
                    s = float(input("  Speed ( -1.0 to 1.0): "))
                    d = float(input("  Duration (seconds): "))
                    node.run_step_test(s, d)
                except ValueError: print("Invalid input.")
            elif cmd == 'sine':
                try:
                    a = float(input("  Amplitude (0.0 to 1.0): "))
                    f = float(input("  Frequency (Hz): "))
                    d = float(input("  Duration (seconds): "))
                    node.stop_event.clear()
                    node.run_sine_test(a, f, d)
                except ValueError: print("Invalid input.")
            else:
                try:
                    val = float(cmd)
                    node.send_speed(val)
                    print(f"Sent constant speed: {val}")
                except ValueError:
                    print("Unknown command or invalid value.")
                    
    except KeyboardInterrupt:
        pass
    finally:
        node.send_speed(0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
