#!/usr/bin/python3
# -*- coding: utf-8 -*-

from std_msgs.msg import Float32
from dolasim_msgs.msg import IcVeriDolasim
from lgsvl_msgs.msg import VehicleControlData
from rclpy.node import Node
import rclpy, time

class SimulationOutput(Node):
    def __init__(self):
        super().__init__('SimOutput')
        self.sub1 = self.create_subscription(IcVeriDolasim, '/v3/main', self.get_data, 10)
        
        self.pub1 = self.create_publisher(VehicleControlData, "/lgsvl/control_input", 10)
        self.pub2 = self.create_publisher(Float32, "/hz_info/output_hz", 10)

        self.get_logger().info("Output started")
        self.output_old_time = time.time()
        self.giris = IcVeriDolasim()
        self.output_hz = Float32()

    def get_data(self, data):
        self.giris.direksiyon = data.direksiyon
        self.giris.gaz = data.gaz
        self.giris.fren = data.fren
        self.giris.vites = data.vites

    def output(self):
        self.out = VehicleControlData()
        self.out.acceleration_pct = self.giris.gaz
        self.out.braking_pct = self.giris.fren
        self.out.target_wheel_angle = self.giris.direksiyon
        self.out.target_gear = self.giris.vites

        self.pub1.publish(self.out)

        self.output_new_time = time.time()
        hz = 1.0 / (self.output_new_time - self.output_old_time)
        self.output_old_time = self.output_new_time
        self.output_hz.data = hz
        
        self.pub2.publish(self.output_hz)

def main(args=None):
    rclpy.init(args=args)
    sim_output = SimulationOutput()
    rate = sim_output.create_rate(30)
    while rclpy.ok():
        try:
            rclpy.spin_once(sim_output)
            sim_output.output()
        except KeyboardInterrupt:
            print("gule gule")

    rclpy.shutdown()

if __name__ == "__main__":
    main(args)