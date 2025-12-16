#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32

class JoyControlROS2(Node):
    def __init__(self):
        super().__init__('joy_control_ros2_sim')

        self.speed_linear = 0.5
        self.speed_angular = 0.5

        self.cmd_vel_pub = self.create_publisher(Twist, '/bluerov/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Bool, '/bluerov/arm_disarm', 10)
        self.luces_pub = self.create_publisher(Int32, '/bluerov/luces_pwm', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info("Logitech F310 Joystick-to-cmd_vel node started")

    def joy_callback(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        twist = Twist()

        # --- Linear motion ---
        twist.linear.x = self.speed_linear * axes[1] if len(axes) > 1 else 0.0   # Forward/backward
        twist.linear.y = -self.speed_linear * axes[0] if len(axes) > 0 else 0.0   # Left/right
        twist.linear.z = self.speed_linear * axes[4] if len(axes) > 4 else 0.0   # Up/down

        # --- Angular motion ---
        twist.angular.z = -self.speed_angular * axes[3] if len(axes) > 3 else 0.0  # Yaw

        # Pitch from triggers
        pitch = 0.0
        if len(axes) > 2:
            pitch -= axes[2]   # LT axis (0->1), invert for down
        if len(axes) > 5:
            pitch += axes[5]   # RT axis (0->1)
        twist.angular.y = self.speed_angular * pitch

        # Roll from bumpers
        roll = 0.0
        if len(buttons) > 4 and buttons[4] == 1:  # LB
            roll -= 1.0
        if len(buttons) > 5 and buttons[5] == 1:  # RB
            roll += 1.0
        twist.angular.x = self.speed_angular * roll

        # Publish cmd_vel
        self.cmd_vel_pub.publish(twist)

        # --- Arming / disarming ---
        if len(buttons) > 7 and buttons[7] == 1:  # Start
            self.arm_pub.publish(Bool(data=True))
            self.get_logger().info("ARMED")
        if len(buttons) > 6 and buttons[6] == 1:  # Back
            self.arm_pub.publish(Bool(data=False))
            self.get_logger().info("DISARMED")

        # --- Lights ---
        if len(axes) > 6:
            if axes[6] == -1:
                self.luces_pub.publish(Int32(data=1900))
                self.get_logger().info("Lights ON")
            elif axes[6] == 1:
                self.luces_pub.publish(Int32(data=0))
                self.get_logger().info("Lights OFF")


def main(args=None):
    rclpy.init(args=args)
    node = JoyControlROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

