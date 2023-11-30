#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped, Float64Stamped
from rclpy.node import Node
from scipy import signal as sig
from collections import deque

class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_filtered_depth = 0.0
        self.current_depth = 0.0
        self.integral = 0.0
        self.start_time = self.get_clock().now().nanoseconds *1e-9
        self.arbeiten_mit_filter = False
        self.time_before= self.get_clock().now().nanoseconds *1e-9  #zeit bei letztem "Tick" für dt
        self.error_before = 0.0                                     #vorheriger Error für derror

        #Reglerperformance:
        self.regler_test = False
        self.max_ueberschwingen = 0.0
        self.rise_time_anfang = 0.0
        self.rise_time_ende = 0.0
        self.rise_time_messung_begonnen = False
        self.rise_time_messung_beendet = False
        self.settling_time_percentage = 0.05
        self.settling_time_anfang = None
        self.settling_time_ende = None
        self.settling_time_found = False
        self.settling_time = None
        self.settling_time_begonnen = False
        


        #Rauschen Amplitude Test
        self.rauschen_test = False
        self.rauschen_max = 0.0
        self.rauschen = 0.0

        



        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(msg_type=Float64Stamped,
                                                     topic='depth_setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=1)
        self.depth_sub = self.create_subscription(msg_type=DepthStamped,
                                                  topic='depth',
                                                  callback=self.on_depth,
                                                  qos_profile=1)
        self.filtered_depth_sub = self.create_subscription(msg_type=DepthStamped, topic='filtered_depth',
                                                           callback=self.on_filtered_depth, qos_profile=1)
        

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        current_depth = depth_msg.depth

        #self.get_logger().info(
        #    f"Hi! I'm your controller running. "
        #    f'I received a depth of {current_depth} m.',
        #    throttle_duration_sec=1)

        thrust = self.compute_control_output(current_depth)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # now = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def on_filtered_depth(self, depth_msg:DepthStamped): #callbackfunktion gefilterte Tiefe
        self.current_filtered_depth = depth_msg.depth


    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float) -> float:
        if self.current_setpoint <= -0.8 and self.current_setpoint >= -0.1:
            return 0.0
 
        else:
            time_now = self.get_clock().now().nanoseconds *1e-9
            dt = time_now - self.time_before
            
            if self.arbeiten_mit_filter == False:
                error = self.current_setpoint - current_depth
            else:
                error = self.current_setpoint - self.current_filtered_depth
            if abs(self.integral) > 10:
                self.integral = 0
            self.integral +=  error *dt
            d_error = (error - self.error_before)/dt
            

            P = 4* error  #4
            I = 0.176* self.integral  #0.176
            D = 4.55* d_error  #4.55
            thrust_z = P + I  + D#This doesn't seem right yet...

            #Reglerperformance (Step): Es wird bei Verwendung des Filters für zB den Überschwingenwert
            if self.regler_test == True:  #das gefilterte Signal verwendet, da ich hoffe, dass dies näher
                vergangene_zeit = time_now - self.start_time   # am realen Wert ist
                if current_depth > -0.4 and abs(error) > self.max_ueberschwingen and self.current_setpoint == -0.4:
                    self.max_ueberschwingen = abs(error)

                if current_depth >= -0.58 and self.rise_time_messung_begonnen == False and self.current_setpoint == -0.4:
                    self.rise_time_messung_begonnen = True
                    self.rise_time_anfang = self.get_clock().now().nanoseconds *1e-9
                
                if current_depth >= -0.42 and self.rise_time_messung_beendet == False and self.rise_time_messung_begonnen == True:
                    self.rise_time_ende = self.get_clock().now().nanoseconds *1e-9
                    self.rise_time_messung_beendet = True

                if self.rise_time_messung_beendet == True:
                    self.get_logger().info(
                        f"rise_time = {self.rise_time_ende-self.rise_time_anfang}",
                        throttle_duration_sec=4)
                if self.current_setpoint == -0.4:
                    if self.settling_time_begonnen == False:
                        self.settling_time_anfang = self.get_clock().now().nanoseconds *1e-9
                        self.settling_time_begonnen = True
                    if abs(error) <= self.settling_time_percentage * abs(self.current_setpoint) and self.settling_time_found == False:
                        self.settling_time_ende = self.get_clock().now().nanoseconds *1e-9
                        self.settling_time = self.settling_time_ende - self.settling_time_anfang
                        self.settling_time_found = True
                    if abs(error) > self.settling_time_percentage * abs(self.current_setpoint) and self.settling_time_found == True:   
                        self.settling_time_ende = None
                        self.settling_time = None
                        self.settling_time_found = False

                self.get_logger().info(
                    f"max_ueberschwingen = {self.max_ueberschwingen}, settling_time = {self.settling_time}",
                    throttle_duration_sec=4)
            
            if self.rauschen_test == True:
                self.rauschen = current_depth - self.current_setpoint
                vergangene_zeit = self.get_clock().now().nanoseconds *1e-9 - self.start_time
                if vergangene_zeit >= 30 and abs(self.rauschen) > abs(self.rauschen_max) :
                    self.rauschen_max = abs(self.rauschen)
                self.get_logger().info(
                    f"max_rauschen = {self.rauschen_max}",
                    throttle_duration_sec=4)

                

            self.time_before = time_now
            self.error_before = error
        
        #self.get_logger().info(
        #    f"Hi! I'm your controller running. "
        #    f'dt = {dt}, error = {error}',
        #    throttle_duration_sec=1)


        return thrust_z

def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
