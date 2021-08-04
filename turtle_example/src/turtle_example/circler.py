import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute


class Circler(Node):
    def __init__(self):
        super().__init__("circler")
        self.vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 1000)
        client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.time_ = self.create_timer(0.1, self.timer_callback)

        srv = TeleportAbsolute.Request()
        srv.x = 2.0
        srv.y = 3.0
        srv.theta = -45.0

        if client.call_async(srv):
            self.get_logger().info(f"Teleporting to {srv.x}, {srv.y}, {srv.theta}")
        else:
            self.get_logger().error("Failed to call service")

    def timer_callback(self):
        vel = Twist()
        vel.linear.x = 2.0
        vel.angular.z = 0.5
        self.vel_pub_.publish(vel)


def main():
    rclpy.init()
    circler = Circler()
    rclpy.spin(circler)
    circler.destroy_node()
    rclpy.shutdown()
