#!/usr/bin/python3
# -*- coding: utf-8 -*-

from dolasim_msgs.msg import IcVeriDolasim
from rclpy.node import Node
from ament_index_python.packages import get_package_share_path as get_path
from cv_bridge import CvBridge
import rclpy, cv2, csv, os

class logger(Node):
    def __init__(self):
        super().__init__("Serit_logger")
        self.sub1 = self.create_subscription(IcVeriDolasim, "/input/main", self.get_main_data, 10)

        self.cvb = CvBridge()
        self.started = False
        self.log_enabled = False
        self.path = self.get_path()
        self.img_count = 0

        self.get_logger().info("Logger started")

    def get_path(self):
        main_file = str(get_path("serit")).split("install")[0] + "src/logger"
        self.get_logger().info("main file = " + main_file)
        path = os.getcwd()
        os.chdir(main_file + '/data')
        data_folder_path = os.getcwd()
        for i in range(0,25):
            if os.path.exists(str(i)) and len(os.listdir(str(i))) == 0:
                data_folder_path += "/" + str(i)
                break
            if not os.path.exists(str(i)):
                os.mkdir(os.path.join(data_folder_path,str(i)))
                data_folder_path += "/" + str(i)
                break
        
        os.chdir(path)
        self.get_logger().info("data folder = " + data_folder_path)
        return data_folder_path

    def get_main_data(self, data):
        self.main_input = data
        self.log_enabled = data.log_enabled

    def get_img_data(self, data):
        try:
            return self.cvb.compressed_imgmsg_to_cv2(data)
        except Exception as e:
            self.get_logger().info("Logger cvb exception")

    def control_csv(self):
        if not os.path.exists(self.path + "/logger.csv"):
            with open(self.path + "/logger.csv", "w") as f:
                writer = csv.writer(f)
                writer.writerow(["Img Name", "direksiyon", "serit_secim", "hiz", "Vites", "gaz", "fren", "guc", "el_freni"])

    def write_csv(self):
        cv2.imwrite(self.path + "/" + str(self.img_count) + ".jpg", self.get_img_data(self.main_input.lane_img))
        self.img_count += 1
        line = [str(self.img_count), str(self.main_input.direksiyon), str(self.main_input.serit_secim), str(self.main_input.anlik_hiz), str(self.main_input.vites),
                str(self.main_input.gaz), str(self.main_input.fren), str(self.main_input.guc), str(self.main_input.el_freni)]
                
        with open(self.path + "/logger.csv", "a") as f:
            writer = csv.writer(f)
            writer.writerow(line)

    def log(self):
        if self.log_enabled and not self.started:
            self.control_csv()
            self.started = True
        
        if self.log_enabled:
            self.write_csv()

def main(args=None):
    rclpy.init(args=args)
    log = logger()
    rate = log.create_rate(30)
    while rclpy.ok():
        try:
            rclpy.spin_once(log)
            log.log()
        except KeyboardInterrupt:
            print("bye bye")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main(args)