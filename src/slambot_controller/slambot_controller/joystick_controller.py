#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class JoystickMapper(Node):
    def __init__(self):
        super().__init__('joystick_mapper')

        # Serial Communication Setup
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value

        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=0.1)

        # ROS Subscriptions & Publishers
        self.subscription = self.create_subscription(
            Joy, '/joystick/joy', self.joy_callback, 10
        )
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_speeds', 10)

        # Store motor speed data for transmission
        self.motor_data = [0.0, 0.0, 0.0, 0.0]

        # Timer to transmit serially at 10 Hz
        self.create_timer(0.1, self.transmit_data_serially)

        self.get_logger().info("Joystick Mapper Node Initialized")

    def joy_callback(self, msg: Joy):
        """Process joystick input and compute motor speeds."""
        left_x = msg.axes[0]  
        left_y = msg.axes[1]  

        left_magnitude = min(1.0, math.sqrt(left_x**2 + left_y**2))  
        left_direction = (math.atan2(left_y, -left_x) * 180.0 / math.pi) % 360.0

        dir1 = 1.0 if 0 <= left_direction <= 180 else 0.0
        dir2 = dir1  

        max_throttle = int(left_magnitude * 255)  

        if left_direction < 90 or left_direction > 270: 
            pwm1 = max_throttle 
            pwm2 = max_throttle * left_magnitude
        else: 
            pwm1 = max_throttle * left_magnitude
            pwm2 = max_throttle 

        self.motor_data = [float(dir1), float(dir2), float(pwm1), float(pwm2)]

        self.get_logger().info(f"Motor Speeds - dir1: {dir1}, dir2: {dir2}, pwm1: {pwm1}, pwm2: {pwm2}")

        # Publish data to ROS
        output = Float32MultiArray(data=self.motor_data)
        self.publisher.publish(output)

    def transmit_data_serially(self):
        """Transmit motor data serially to ESP32."""
        data_string = ','.join(f"{value:.2f}" for value in self.motor_data) + '\n'
        self.serial_connection.write(data_string.encode('utf-8'))
        self.get_logger().info(f"Transmitted to ESP32: {data_string.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
