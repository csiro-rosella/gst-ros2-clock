import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock


class ClockPublisher(Node):

    NS_PER_S = 1000000000

    def __init__(self, rate_hz=100, start_ns=0, step_ns=10000000):
        super().__init__("clock_publisher")
        self.publisher_ = self.create_publisher(Clock, "/clock", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.time_ns = start_ns
        self.step_ns = step_ns

    def timer_callback(self):
        msg = Clock()
        msg.clock.sec = int(self.time_ns // self.NS_PER_S)
        msg.clock.nanosec = int(self.time_ns % self.NS_PER_S)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.clock)
        self.time_ns += self.step_ns


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser()

    DEFAULT_RATE = 100
    parser.add_argument(
        "--pub-rate",
        default=DEFAULT_RATE,
        type=int,
        help="Rate at which to publish clock messages",
    )
    parser.add_argument(
        "--start-time", default=0, type=int, help="Initial clock time in nanoseconds"
    )
    parser.add_argument(
        "--time-step",
        default=1000000000 / DEFAULT_RATE,
        type=int,
        help="Time increment in nanoseconds",
    )

    args = parser.parse_args()

    rclpy.init()

    clock_publisher = ClockPublisher(args.pub_rate, args.start_time, args.time_step)

    try:
        rclpy.spin(clock_publisher)
    except KeyboardInterrupt:
        pass

    clock_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
