import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image


class DualMonoSyncCheck(Node):
    def __init__(self):
        super().__init__('dual_mono_sync_check')
        self.declare_parameter('left_topic', '/left/left/image_raw')
        self.declare_parameter('right_topic', '/right/right/image_raw')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop_s', 0.02)
        self.declare_parameter('report_every', 60)

        left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        slop_s = self.get_parameter('slop_s').get_parameter_value().double_value
        report_every = self.get_parameter('report_every').get_parameter_value().integer_value

        self._report_every = max(1, int(report_every))
        self._count = 0
        self._sum_abs_dt = 0.0
        self._max_abs_dt = 0.0

        self.get_logger().info(
            f'Sync check subscribing to {left_topic} and {right_topic} '
            f'(slop={slop_s:.3f}s, queue={queue_size})'
        )

        left_sub = Subscriber(self, Image, left_topic)
        right_sub = Subscriber(self, Image, right_topic)
        ats = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size, slop_s)
        ats.registerCallback(self._on_pair)

    def _on_pair(self, left_msg: Image, right_msg: Image):
        left_t = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec * 1e-9
        right_t = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec * 1e-9
        abs_dt = abs(left_t - right_t)

        self._count += 1
        self._sum_abs_dt += abs_dt
        self._max_abs_dt = max(self._max_abs_dt, abs_dt)

        if self._count % self._report_every == 0:
            avg_dt = self._sum_abs_dt / self._count
            self.get_logger().info(
                f'Pairs={self._count} avg_dt={avg_dt*1000:.2f}ms '
                f'max_dt={self._max_abs_dt*1000:.2f}ms'
            )


def main():
    rclpy.init()
    node = DualMonoSyncCheck()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
