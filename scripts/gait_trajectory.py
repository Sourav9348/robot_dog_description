#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from gait_patterns import JOINT_NAMES, get_gait


class GaitTrajectoryNode(Node):
    def __init__(self):
        super().__init__("gait_trajectory")

        self.declare_parameter("gait_type", "trot")
        gait_type = self.get_parameter("gait_type").get_parameter_value().string_value

        self.get_logger().info(
            f"gait_trajectory node started. Use 'gait_type' param: trot / pace / bound. Current: {gait_type}"
        )

        self.duration = 4.0   
        self.dt = 0.02

        self.t, self.traj = get_gait(gait_type, self.duration, self.dt)
        self.n_steps = len(self.t)
        self.idx = 0

        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()

        for name in JOINT_NAMES:
            point.positions.append(float(self.traj[name][self.idx]))

        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))

        msg.points.append(point)

        self.pub.publish(msg)

        self.idx = (self.idx + 1) % self.n_steps


def main(args=None):
    rclpy.init(args=args)
    node = GaitTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
