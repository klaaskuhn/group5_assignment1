#!/usr/bin/env python3
"""
This node takes as input the pressure data and computes a resulting water depth.
"""
import rclpy
from hippo_msgs.msg import DepthStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import FluidPressure
import random as rnd


class DepthCalculator(Node):

    def __init__(self):
        super().__init__(node_name='depth_calculator')

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.depth_pub = self.create_publisher(msg_type=DepthStamped,
                                               topic='depth',
                                               qos_profile=1)
        self.pressure_sub = self.create_subscription(msg_type=FluidPressure,
                                                     topic='pressure',
                                                     callback=self.on_pressure,
                                                     qos_profile=qos)
        self.offset = 0.0
        self.p_umgebung = 100000
        

    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        pressure = pressure_msg.fluid_pressure
        depth = self.pressure_to_depth(pressure=pressure)
        now = self.get_clock().now()
        self.publish_depth_msg(depth=depth, now=now)

    def publish_depth_msg(self, depth: float, now: rclpy.time.Time) -> None:
        msg = DepthStamped()
        # Let's add a time stamp
        msg.header.stamp = now.to_msg()
        # and populate the depth field
        msg.depth = depth
        self.depth_pub.publish(msg)

    def pressure_to_depth(self, pressure: float) -> float:
        # TODO: implement the required depth calculation
        depth = -(pressure-self.p_umgebung)/(1000*9.81)  
        return depth +self.offset #+ rnd.uniform(-0.01, 0.01)


def main():
    rclpy.init()
    node = DepthCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
