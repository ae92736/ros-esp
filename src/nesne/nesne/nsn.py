#!/usr/bin/python3
# -*- coding: utf-8 -*-

from dolasim_msgs.msg import IcVeriDolasim
from std_msgs.msg import Bool, Float32, String
from rclpy.node import Node
from cv_bridge import CvBridge
from . import darknet
import numpy as np
import cv2, random, time, rclpy

class nesne(Node):
    def __init__(self):
        super().__init__("nesne")
        self.sub1 = self.create_subscription(IcVeriDolasim, "/input/main", self.get_main_data, 10)
        self.sub2 = self.create_subscription(Bool, "/feedback/nesne_tespiti", self.get_working_state, 10)

        self.main_input = IcVeriDolasim()
        self.state = False

        self.pub1 = self.create_publisher(IcVeriDolasim, "/v1/main", 10)
        self.pub2 = self.create_publisher(Float32, "/hz_info/nesne_tespiti", 10)
        self.pub3 = self.create_publisher(Float32, "/hz_info/yolo_inferance_time", 10)
        self.pub4 = self.create_publisher(String, "/v1/yonelim", 10)

        self.output_time_old = time.time()
        self.output_time = Float32()
        self.output_time.data = 0.0
        self.model_time = Float32()
        self.model_time.data = 0.0

        self.config_file = "/home/aye/yolov4/rb22/rb22_test.cfg"
        self.data_file = "/home/aye/yolov4/rb22/rb22.data"
        self.weights_file = "/home/aye/yolov4/backup/rb22_last.weights"

        #self.net, self.classes, self.colors = self.get_model()
        self.cvb = CvBridge()

        self.get_logger().info("Nesne Tespiti started")
        self.create_timer(0.03, self.output)

    def get_model(self):
        random.seed(5)
        net, clss, color = darknet.load_network(self.config_file, self.data_file, self.weights_file)

        return net, clss, color

    def get_main_data(self, data):
        self.main_input = data

    def get_working_state(self, data):
        self.state = data.data

    def get_img_data(self, data):
        try:
            img = self.cvb.compressed_imgmsg_to_cv2(data)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            return img

        except exception as e:
            print("nesne cvb error", e)

    def get_compressed_image(self, data):
        try:
            img = self.cvb.cv2_to_compressed_imgmsg(data, "jpeg")
            return img
        except Exception as e:
            print("nesne compression error", e)

    def run_model(self, data):
        st = time.time()

        img = self.get_img_data(data.obj_img)

        darknet_image = darknet.make_image(1280, 720, 3)

        image_resized = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image, img.tobytes())
        detections = darknet.detect_image(self.net, self.classes, darknet_image, thresh=0.7)
        darknet.free_image(darknet_image)
        img = darknet.draw_boxes(detections, image_resized, self.colors)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        ot = time.time()
        self.model_time.data = 1.0 / (ot - st)
        self.pub3.publish(self.model_time)

        return self.get_compressed_image(img)

    def output(self):
        self.output_time_new = time.time()
        hz = 1.0 / (self.output_time_new - self.output_time_old)
        self.output_time_old = self.output_time_new
        self.output_time.data = hz
        
        if self.state:
            self.main_input.obj_img = self.run_model(self.main_input)

        self.pub1.publish(self.main_input)
        self.pub2.publish(self.output_time)

def main(args=None):
    rclpy.init(args=args)
    seer = nesne()
    rate = seer.create_rate(30)
    while rclpy.ok():
        try:
            rclpy.spin_once(seer)
            seer.output()
        except KeyboardInterrupt:
            print("good bye")

    rclpy.shutdown()

if __name__ == "__main__":
    main(args)