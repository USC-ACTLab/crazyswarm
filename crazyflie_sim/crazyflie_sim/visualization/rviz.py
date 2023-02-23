from __future__ import annotations

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from ..sim_data_types import State, Action


class Visualization:
    """Publishes ROS 2 transforms of the states, so that they can be visualized in RVIZ"""

    def __init__(self, node: Node, params: dict, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.tfbr = TransformBroadcaster(self.node)

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        # publish transformation to visualize in rviz
        msgs = []
        for name, state in zip(self.names, states):
            msg = TransformStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.child_frame_id = name
            msg.transform.translation.x = state.pos[0]
            msg.transform.translation.y = state.pos[1]
            msg.transform.translation.z = state.pos[2]
            msg.transform.rotation.x = state.quat[1]
            msg.transform.rotation.y = state.quat[2]
            msg.transform.rotation.z = state.quat[3]
            msg.transform.rotation.w = state.quat[0]
            msgs.append(msg)
        self.tfbr.sendTransform(msgs)

    def shutdown(self):
        pass

