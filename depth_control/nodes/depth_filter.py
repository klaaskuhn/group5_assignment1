#!/usr/bin/env python3

import rclpy
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped, Float64Stamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import signal as sig

class depth_filter(Node):
    def __init__(self):
        super().__init__(node_name = 'depth_filter')

        self.depth_before = None
        self.alp = 0.07 #0.07
        self.depth_sub = self.create_subscription(msg_type= DepthStamped, topic = 'depth', 
                                                  callback= self.on_depth, qos_profile=1)
        self.filtered_depth_pub = self.create_publisher(msg_type=DepthStamped,
                                                         topic='filtered_depth', qos_profile=1)

    def on_depth(self, depth_msg:DepthStamped):
        current_depth = depth_msg.depth
        filtered_depth = self.filters(current_depth)
        now = self.get_clock().now()
        self.publish_filtered_depth(filtered_depth, timestamp=now)
    
    def publish_filtered_depth(self, filtered_depth:float, timestamp:rclpy.time.Time) -> None:
        msg = DepthStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.depth = filtered_depth
        self.filtered_depth_pub.publish(msg)

    
    
    def filters(self, current_depth: float) -> float:
        if self.depth_before == None:
            self.depth_before = current_depth
            return current_depth
        else:
            filtered_depth = self.alp * current_depth + (1-self.alp) * self.depth_before
            self.depth_before = filtered_depth
            return filtered_depth
def main():
    rclpy.init()
    Node = depth_filter()
    try: 
        rclpy.spin(Node)
    except KeyboardInterrupt:
        pass
if __name__ == '__main__':
    main()

        