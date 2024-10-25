import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32


class PS4ControlNode(Node):
    def __init__(self):
        super().__init__('ps4_control_node')
        
        # Subscriptions
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Publishers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.buzzer_publisher = self.create_publisher(Bool, 'Buzzer', 10)
        self.rgb_publisher = self.create_publisher(Int32, 'RGBLight', 10)

        # Initial values
        self.speed = 0.0
        self.steering = 0.0
        self.linear_gear = 1.0
        self.angular_gear = 1.0
        self.buzzer_active = False
        self.rgb_light_index = 0

    def joy_callback(self, msg):
        # Handle gear control
        if msg.buttons[4]:  # L1 button
            self.linear_gear = self.cycle_gear(self.linear_gear, [1.0, 0.33, 0.67])
            self.get_logger().info(f'Linear gear changed to: {self.linear_gear}')
        
        if msg.buttons[5]:  # R1 button
            self.angular_gear = self.cycle_gear(self.angular_gear, [1.0, 0.25, 0.5, 0.75])
            self.get_logger().info(f'Angular gear changed to: {self.angular_gear}')
        
        # Handle buzzer toggle
        if msg.buttons[0]:  # X button
            self.buzzer_active = not self.buzzer_active
            self.buzzer_publisher.publish(Bool(data=self.buzzer_active))
            self.get_logger().info(f'Buzzer {"on" if self.buzzer_active else "off"}')
        
        # Handle RGB light cycling
        if msg.buttons[3]:  # Square button
            self.rgb_light_index = (self.rgb_light_index + 1) % 7  # Cycle through 0-6
            self.rgb_publisher.publish(Int32(data=self.rgb_light_index))
            self.get_logger().info(f'RGB Light set to index: {self.rgb_light_index}')

        # Read joystick and trigger values
        forward = (1 - msg.axes[5]) / 2  # R2 is axis 5, normalize to 0-1 with 1 being fully pressed
        reverse = (1 - msg.axes[2]) / 2  # L2 is axis 4, normalize to 0-1 with 1 being fully pressed
        steering = msg.axes[0]  # Left joystick horizontal is axis 0

        # Ensure only one of forward or reverse is active at a time
        if forward > 0.1 and reverse > 0.1:  # Adjust threshold if needed
            if forward > reverse:
                self.speed = forward
            else:
                self.speed = -reverse
        elif forward > 0.1:
            self.speed = forward
        elif reverse > 0.1:
            self.speed = -reverse
        else:
            self.speed = 0.0

        # Apply the gears
        self.speed *= self.linear_gear
        self.steering = steering * self.angular_gear

        # Apply dead zone
        self.steering = self.filter_deadzone(self.steering)
        self.speed = self.filter_deadzone(self.speed)

        # Publish the twist message
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.steering
        self.publisher.publish(twist)

        # Log the twist message for debugging
        self.get_logger().info(f'Publishing Twist: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}')

    def filter_deadzone(self, value, threshold=0.2):
        """Filter out small joystick movements."""
        return 0.0 if abs(value) < threshold else value

    def cycle_gear(self, current_value, gear_values):
        """Cycle through predefined gear values."""
        index = gear_values.index(current_value)
        return gear_values[(index + 1) % len(gear_values)]


def main(args=None):
    rclpy.init(args=args)
    node = PS4ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
