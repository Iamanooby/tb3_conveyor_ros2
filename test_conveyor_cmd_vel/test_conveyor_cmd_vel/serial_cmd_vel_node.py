import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class SerialCmdVelNode(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel_node')

        # Serial port configuration
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 57600
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Serial connection established on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            raise

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            # 'cmd_vel_safe',
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Subscribed to cmd_vel topic")

        # Timer to send serial data at 0.1-second intervals
        self.timer = self.create_timer(0.2, self.send_serial_data)
        self.latest_cmd_vel = Twist()  # Store the latest received Twist message

    def cmd_vel_callback(self, msg: Twist):
        # Update the latest_cmd_vel with the received message
        self.latest_cmd_vel = msg
        self.get_logger().info(f"Received cmd_vel: linear_x={msg.linear.x}, linear_y={msg.linear.y}, angular_z={msg.angular.z}")

    def send_serial_data(self):
        # Extract the latest velocities
        linear_x = self.latest_cmd_vel.linear.x
        linear_y = self.latest_cmd_vel.linear.y
        angular_z = self.latest_cmd_vel.angular.z

        # Format the string to be sent via serial
        serial_data = f"{linear_x:.3f},{linear_y:.3f},{angular_z:.3f}\n"

        # Send the data over serial
        try:
            self.serial_connection.write(serial_data.encode())
            self.get_logger().info(f"Sent to serial: {serial_data.strip()}")

            if self.serial_connection.in_waiting > 0:  # Check if data is available to read
                response = self.serial_connection.readline().decode().strip()  # Read a line and decode
                self.get_logger().info(f"Received from serial: {response}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial: {e}")

    def destroy_node(self):
        # Clean up serial connection
        if self.serial_connection.is_open:
            self.serial_connection.close()
        self.get_logger().info("Serial connection closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
