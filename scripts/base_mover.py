#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BaseMover(Node):

    def __init__(self):
        super().__init__('base_mover')

        self.br = TransformBroadcaster(self)
        self.dt = 0.02      
        self.timer = self.create_timer(self.dt, self.update)

        self.t = 0.0
        self.v = 0.1        
        self.z = 0.10       
        self.yaw_rate = 0.0 

    def update(self):
        self.t += self.dt
        x = self.v * self.t
        y = 0.0
        z = self.z

        yaw = self.yaw_rate * self.t

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'

        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z

        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.br.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
