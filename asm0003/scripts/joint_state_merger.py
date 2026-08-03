#!/usr/bin/env python3
"""Merge multiple sensor_msgs/JointState topics into one."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def parse_topics(value):
    """Parse comma/space separated topic list or YAML list string."""
    if not value:
        return []
    value = value.strip()
    if value.startswith('[') and value.endswith(']'):
        value = value[1:-1]
    return [t.strip().strip('"\'') for t in value.split(',') if t.strip()]


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        self.declare_parameter('input_topics', '/master2/joint_states,/master0/joint_states')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 100.0)

        input_topics_raw = self.get_parameter('input_topics').get_parameter_value().string_value
        input_topics = parse_topics(input_topics_raw)
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self._latest = {}
        self._subs = []
        for topic in input_topics:
            self._subs.append(
                self.create_subscription(JointState, topic, self._on_joint_state, 10)
            )
            self.get_logger().info(f'Subscribed to {topic}')

        self._pub = self.create_publisher(JointState, output_topic, 10)
        period = 1.0 / max(1.0, publish_rate)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info(f'Publishing merged joint states to {output_topic}')

    def _on_joint_state(self, msg):
        has_velocity = len(msg.velocity) == len(msg.name)
        has_effort = len(msg.effort) == len(msg.name)
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 0.0
            velocity = msg.velocity[i] if has_velocity else 0.0
            effort = msg.effort[i] if has_effort else 0.0
            self._latest[name] = (position, velocity, effort)

    def _publish(self):
        if not self._latest:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in sorted(self._latest.keys()):
            pos, vel, eff = self._latest[name]
            msg.name.append(name)
            msg.position.append(pos)
            msg.velocity.append(vel)
            msg.effort.append(eff)
        self._pub.publish(msg)
        self.get_logger().debug(
            f'Published merged {len(msg.name)} joints',
            throttle_duration_sec=2.0,
        )
    


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
