#!/usr/bin/python3
# -*- coding: utf-8 -*-

from dolasim_msgs.msg import IcVeriDolasim
from std_msgs.msg import Bool, Float32
from rclpy.node import Node
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_path as get_path
import numpy as np
import rclpy, cv2, time
class serit_takibi(Node):
    def __init__(self):
        super().__init__("Serit_Takibi")
        self.sub1 = self.create_subscription(IcVeriDolasim, "/v2/main", self.get_main_data, 10)
        self.sub2 = self.create_subscription(Bool, "/feedback/serit_takibi", self.get_working_state, 10)
        
        self.main_input = IcVeriDolasim()
        self.state = False

        self.pub1 = self.create_publisher(IcVeriDolasim, "/v3/main", 10)
        self.pub2 = self.create_publisher(Float32, "/hz_info/serit_takibi", 10)
        self.pub3 = self.create_publisher(Float32, "/hz_info/cnn_inferance_time", 10)

        self.f_name = str(get_path("serit")).split("install")[0] + "src/serit/serit/logger.csv"
        self.lines = None
        self.read_all_lines()
        self.length = len(self.lines)
        self.count = 0

        self.output_time_old = time.time()
        self.output_time = Float32()
        self.output_time.data = 0.0
        self.model_time = Float32()
        self.model_time.data = 0.0

        self.cvb = CvBridge()
        self.get_logger().info("Serit Takibi started")

    def read_all_lines(self):
        self.lines = np.genfromtxt(self.f_name, delimiter=",", skip_header=True, dtype=None, encoding='UTF-8')

    def get_main_data(self, data):
        self.main_input = data
        if self.state:
            self.image = self.get_img_data(data.lane_img)

    def get_working_state(self, data):
        self.state = data.data

    def get_img_data(self, data):
        try:
            img = self.cvb.compressed_imgmsg_to_cv2(data)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img
        except Exception as e:
            print("serit takibi cvb error")
            print(e)

    def run_model(self):
        st = time.time()
        ot = time.time()

        self.count += 1

        self.model_time.data = 30.25678
        self.pub3.publish(self.model_time)
        
        return self.main_input.direksiyon, self.main_input.gaz, self.main_input.fren

    def output(self):
        self.output_time_new = time.time()
        hz = 1.0 / (self.output_time_new - self.output_time_old)
        self.output_time_old = self.output_time_new
        self.output_time.data = hz

        if self.state:
            d, f, g = self.run_model()
            self.main_input.direksiyon = d
            self.main_input.gaz = g
            self.main_input.fren = f

        self.pub1.publish(self.main_input)
        self.pub2.publish(self.output_time)

def main(args=None):
    rclpy.init(args=args)
    tracer = serit_takibi()
    rate = tracer.create_rate(30)
    while rclpy.ok():
        try:
            rclpy.spin_once(tracer)
            tracer.output()
        except KeyboardInterrupt:
            print("good bye")

    rclpy.shutdown()

if __name__ == "__main__":
    main(args)