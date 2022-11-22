from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time


class BackendRviz:

    def __init__(self, node: Node):
        self.node = node
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.tfbr = TransformBroadcaster(self.node)
        self.t = 0
        self.dt = 0.1

    def init(self, names, positions):
        self.names = names

    def time(self) -> float:
        return self.t

    def step(self, states):
        # print("step ", states)

        # advance the time
        self.t += self.dt

        # step the robot itself (here: we just visualize the desired state)
        msgs = []
        for name, state in zip(self.names, states):
            # print(state, type(state.pos[0]))
            msg = TransformStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.child_frame_id = name
            msg.transform.translation.x = state.pos[0]
            msg.transform.translation.y = state.pos[1]
            msg.transform.translation.z = state.pos[2]
            msg.transform.rotation.x = 0.0
            msg.transform.rotation.y = 0.0
            msg.transform.rotation.z = 0.0
            msg.transform.rotation.w = 1.0
            msgs.append(msg)
        self.tfbr.sendTransform(msgs)

        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.time()).to_msg()
        self.clock_publisher.publish(clock_message)

