#!/usr/bin/python3
# -*- coding: utf-8 -*-

from std_msgs.msg import Float32, Int8, Bool
from sensor_msgs.msg import CompressedImage, Joy, LaserScan
from dolasim_msgs.msg import IcVeriDolasim
from lgsvl_msgs.msg import VehicleOdometry
from rclpy.node import Node
from simple_pid import PID
import rclpy, time

axes = {"direksiyon" : 2, "gaz-fren" : 1, "sag-sol": 4, "ileri": 5}
buttons = {"ileri": 3, "geri": 1, "el-freni": 2, "guc": 0}

class SimulationInput(Node):
    def __init__(self):
        super().__init__('SimInput')
        self.sub1 = self.create_subscription(CompressedImage, '/lgsvl/lane_img_output', self.get_lane_img_data, 10)
        self.sub2 = self.create_subscription(CompressedImage, '/lgsvl/obj_img_output', self.get_obj_img_data, 10)
        self.sub3 = self.create_subscription(VehicleOdometry, '/lgsvl/velocity', self.get_vel_data, 10)
        self.sub4 = self.create_subscription(LaserScan, '/lgsvl/lidar', self.get_lidar_data, 10)
        self.sub5 = self.create_subscription(Joy, "/joy", self.get_joy_input, 10)

        self.lane_img_data = CompressedImage()
        self.obj_img_data = CompressedImage()
        self.vel_data = VehicleOdometry()
        self.lidar_data = LaserScan()

        self.direksiyon = 0.0
        self.gaz = 0.0
        self.fren = 0.0
        self.vites = 1

        self.sub6 = self.create_subscription(Bool, "/feedback/guc", self.get_guc, 10)
        self.sub7 = self.create_subscription(Bool, "/feedback/el_freni", self.get_el_freni, 10)
        self.sub8 = self.create_subscription(Int8, "/feedback/serit_secim", self.get_serit_secim, 10)
        self.sub9 = self.create_subscription(Bool, "/feedback/logger", self.get_logger_state, 10)
        self.sub10 = self.create_subscription(Float32, "/feedback/hedef_hiz", self.get_hedef_hiz, 10)
        self.sub11 = self.create_subscription(Bool, "/feedback/joy_enable", self.get_joy_enable, 10)

        self.guc_enabled = True
        self.el_freni = False
        self.serit_secim = 0
        self.log_enabled = False
        self.hedef_hiz = 0
        self.joy_enable = False

        self.pub1 = self.create_publisher(IcVeriDolasim, "/input/main", 10)

        self.pub2 = self.create_publisher(Float32, "/hz_info/svl_lane_img_input", 10)
        self.pub3 = self.create_publisher(Float32, "/hz_info/svl_obj_img_input", 10)
        self.pub4 = self.create_publisher(Float32, "/hz_info/svl_vel_input", 10)
        self.pub5 = self.create_publisher(Float32, "/hz_info/svl_lidar_input", 10)
        self.pub6 = self.create_publisher(Float32, "/hz_info/joy", 10)

        self.lane_img_input_time_old = time.time()
        self.obj_img_input_time_old = time.time()
        self.vel_input_time_old = time.time()
        self.lidar_input_time_old = time.time()
        self.joy_input_time_old = time.time()

        self.lane_img_input_hz = Float32()
        self.obj_img_input_hz = Float32()
        self.vel_input_hz = Float32()
        self.lidar_input_hz = Float32()
        self.joy_input_hz = Float32()

        self.pid = PID(1, 0.5, 0.1, setpoint=2,output_limits=(-10,10))

        self.get_logger().info("Input started")

    def get_lane_img_data(self, data):
        self.lane_img_data = data

        self.lane_img_input_time_new = time.time()
        hz = 1.0 / (self.lane_img_input_time_new - self.lane_img_input_time_old)
        self.lane_img_input_time_old = self.lane_img_input_time_new
        self.lane_img_input_hz.data = hz
        
        self.pub2.publish(self.lane_img_input_hz)

    def get_obj_img_data(self, data):
        self.obj_img_data = data

        self.obj_img_input_time_new = time.time()
        hz = 1.0 / (self.obj_img_input_time_new - self.obj_img_input_time_old)
        self.obj_img_input_time_old = self.obj_img_input_time_new
        self.obj_img_input_hz.data = hz

        self.pub3.publish(self.obj_img_input_hz)

    def get_vel_data(self, data):
        self.vel_data = data

        self.vel_input_time_new = time.time()
        hz = 1.0 / (self.vel_input_time_new - self.vel_input_time_old)
        self.vel_input_time_old = self.vel_input_time_new
        self.vel_input_hz.data = hz

        self.pub4.publish(self.vel_input_hz)

    def get_lidar_data(self, data):
        self.lidar_data = data

        self.lidar_input_time_new = time.time()
        hz = 1.0 / (self.lidar_input_time_new - self.lidar_input_time_old)
        self.lidar_input_time_old = self.lidar_input_time_new
        self.lidar_input_hz.data = hz

        self.pub5.publish(self.lidar_input_hz)

    def get_joy_enable(self, data):
        self.joy_enable = data.data

    def get_joy_input(self, data):
        self.direksiyon = -data.axes[axes["direksiyon"]]

        if data.axes[axes["gaz-fren"]] >= 0:
            self.fren = 0.0
            self.gaz = data.axes[axes["gaz-fren"]]

        else:
            self.gaz = 0.0
            self.fren = -data.axes[axes["gaz-fren"]]
        
        if data.axes[axes["sag-sol"]] > 0:
            self.serit_secim = 2
        
        elif data.axes[axes["sag-sol"]] < 0:
            self.serit_secim = 1

        if data.axes[axes["ileri"]] > 0:
            self.serit_secim = 0

        if data.buttons[buttons["ileri"]]:
            self.vites = 1

        elif data.buttons[buttons["geri"]]:
            self.vites = 2

        elif data.buttons[buttons["guc"]]:
            if self.guc_enabled:    
                self.guc_enabled = False
            else:
                self.guc_enabled = True

        elif data.buttons[buttons["el-freni"]]:
            if self.el_freni:
                self.el_freni = False

            else:
                self.el_freni = True

        self.joy_input_time_new = time.time()
        hz = 1.0 / (self.joy_input_time_new - self.joy_input_time_old)
        self.joy_input_time_old = self.joy_input_time_new
        self.joy_input_hz.data = hz
        
        self.pub6.publish(self.joy_input_hz)

    def get_guc(self, data):
        self.guc_toggled = data.data

    def get_el_freni(self, data):
        self.el_freni_toggled = data.data

    def get_serit_secim(self, data):
        self.serit_secim = data.data

    def get_logger_state(self, data):
        self.log_enabled = data.data

    def get_hedef_hiz(self, data):
        self.hedef_hiz = data.data    

    def check_cruise_control(self):
        if self.hedef_hiz > 0 and self.fren == 0.0:
            self.pid.setpoint = self.hedef_hiz
            out = self.pid(self.vel_data.velocity)
            if out >= 0:
                gaz, fren = (float(out / 10), 0.0)
            else:
                gaz, fren = (0.0, float(out / -10))
            return (gaz, fren)
        
        else:
            self.pid.setpoint = 0
            out = self.pid(self.vel_data.velocity)
            if out >= 0:
                gaz, fren = (float(out / 10), 0.0)
            else:
                gaz, fren = (0.0, float(out / -10))
            return (gaz, fren)

    def output(self):
        self.out = IcVeriDolasim()
        self.out.lane_img = self.lane_img_data
        self.out.obj_img = self.obj_img_data
        self.out.lidar = self.lidar_data

        self.out.serit_secim = self.serit_secim
        self.out.vites = self.vites

        self.out.direksiyon = self.direksiyon
        if not self.joy_enable:
            self.out.gaz, self.out.fren = self.check_cruise_control()
        else:
            self.out.gaz, self.out.fren = self.gaz, self.fren
        self.out.anlik_hiz = self.vel_data.velocity

        self.out.guc = self.guc_enabled
        self.out.el_freni = self.el_freni
        self.out.log_enabled = self.log_enabled

        self.pub1.publish(self.out)
        
def main(args=None):
    rclpy.init(args=args)
    sim_input = SimulationInput()
    rate = sim_input.create_rate(30)
    while rclpy.ok():
        try:
            rclpy.spin_once(sim_input)
            sim_input.output()
        except KeyboardInterrupt:
            print("gule gule")

    rclpy.shutdown()

if __name__ == "__main__":
    main(args)