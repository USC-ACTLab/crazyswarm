from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node


class BackendRviz:

    def __init__(self, node: Node):
        self.node = node
        self.tfbr = TransformBroadcaster(self.node)

    def init(self, names, positions):
        self.names = names

    def step(self, states):
        print("step ", states)

        msgs = []
        for name, state in zip(self.names, states):
            print(state, type(state.pos[0]))
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

