#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class SerialTransmitter(Node):
    def __init__(self):
        super().__init__('serial_transmitter')
        
        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("retry_interval", 1.0)
        self.declare_parameter("max_retries", 3)
        
        # Initialize serial connection
        self.serial_port = None
        self.init_serial()
        
        # Subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speeds',
            self.motor_speed_callback,
            10
        )
        
        self.get_logger().info("Serial Transmitter Node Initialized")

    def init_serial(self):
        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        retry_interval = self.get_parameter("retry_interval").value
        max_retries = self.get_parameter("max_retries").value
        
        for attempt in range(max_retries):
            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=0.1,
                    write_timeout=0.1
                )
                self.get_logger().info(f"Successfully connected to {port}")
                return
            except serial.SerialException as e:
                self.get_logger().warning(
                    f"Attempt {attempt + 1}/{max_retries}: Could not open {port} - {str(e)}")
                if attempt < max_retries - 1:
                    time.sleep(retry_interval)
        
        self.get_logger().error(f"Failed to connect to {port} after {max_retries} attempts")
        self.serial_port = None

    def motor_speed_callback(self, msg):
        if not self.serial_port:
            self.get_logger().warn("Serial port not available, skipping command")
            return
            
        if len(msg.data) != 4:
            self.get_logger().error(f"Expected 4 values, got {len(msg.data)}")
            return
            
        try:
            # Clamp and format values
            dir1 = 1.0 if msg.data[0] > 0.5 else 0.0
            dir2 = 1.0 if msg.data[1] > 0.5 else 0.0
            pwm1 = max(0.0, min(255.0, msg.data[2]))
            pwm2 = max(0.0, min(255.0, msg.data[3]))
            
            # Format message
            cmd = f"{dir1:.1f},{dir2:.1f},{pwm1:.1f},{pwm2:.1f}\n"
            
            # Send command
            self.serial_port.write(cmd.encode('ascii'))
            self.serial_port.flush()
            
            self.get_logger().info(f"Sent command: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {str(e)}")
            # Attempt to reconnect
            self.init_serial()

def main(args=None):
    rclpy.init(args=args)
    node = SerialTransmitter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()