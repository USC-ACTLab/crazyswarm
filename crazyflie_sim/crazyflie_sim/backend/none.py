from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time


class Backend:

    def __init__(self, node: Node):
        self.node = node
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 0.1

    def init(self, names, positions):
        self.names = names

    def time(self) -> float:
        return self.t

    def step(self, setpoints):
        # advance the time
        self.t += self.dt

        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.time()).to_msg()
        self.clock_publisher.publish(clock_message)

