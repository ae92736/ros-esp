#!/usr/bin/python3
# -*- coding: utf-8 -*-

from paho.mqtt import client as mqtt
from std_msgs.msg import Bool, Float32, Int8
from dolasim_msgs.msg import IcVeriDolasim
from rclpy.node import Node
import numpy as np
import os, sys, rclpy, time, cv2, json

class Mqtt_output(Node):
    def __init__(self):
        super().__init__("MqttOut")
        self.sub1 = self.create_subscription(IcVeriDolasim, '/v3/main', self.get_main_data, 10)
        self.sub2 = self.create_subscription(Float32, "/hz_info/joy", self.get_joy_input_hz, 10)
        self.sub3 = self.create_subscription(Float32, "/hz_info/svl_lane_img_input", self.get_svl_lane_img_input_hz, 10)
        self.sub4 = self.create_subscription(Float32, "/hz_info/svl_obj_img_input", self.get_svl_obj_img_input_hz, 10)
        self.sub5 = self.create_subscription(Float32, "/hz_info/svl_vel_input", self.get_velocity_input_hz, 10)
        self.sub6 = self.create_subscription(Float32, "/hz_info/output_hz", self.get_io_output_hz, 10)
        self.sub7 = self.create_subscription(Float32, "/hz_info/serit_takibi", self.get_serit_takibi_hz, 10)
        self.sub8 = self.create_subscription(Float32, "/hz_info/cnn_inferance_time", self.get_serit_inference_time, 10)
        self.sub9 = self.create_subscription(Float32, "/hz_info/nesne_tespiti", self.get_nesne_tespiti_hz, 10)
        self.sub10 = self.create_subscription(Float32, "/hz_info/yolo_inferance_time", self.get_nesne_inference_time, 10)
        self.sub11 = self.create_subscription(Float32, "/feedback/hedef_hiz", self.get_hedef_hiz, 10)
        
        self.direksiyon = 0
        self.gaz = 0
        self.fren = 0
        self.anlik_hiz = 0
        self.hedef_hiz = 0

        self.serit_secim = 0
        self.guc = True
        self.el_freni = False

        self.joy_hz = 0
        self.lane_img_input_hz = 0
        self.obj_img_input_hz = 0
        self.velocity_input_hz = 0
        self.io_output_hz = 0
        self.serit_takibi_hz = 0
        self.serit_inference_time = 0
        self.nesne_tespiti_hz = 0
        self.nesne_inference_time = 0

        self.get_logger().info("mqtt -> ros is ok, starting mqtt")

        self.broker_ip = "192.168.43.195"
        self.broker_port = 1883
        self.id = "sim_mqtt_output"

        self.client = self.connect_mqtt()

    def connect_mqtt(self):
        client = mqtt.Client(self.id)
        client.on_connect = self.on_connect
        client.connect(self.broker_ip, self.broker_port)
        return client

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("mqtt -> connected to broker on" + self.broker_ip + ":" + self.broker_port)
        
        else:
            self.get_logger().info("mqtt -> broker connection failed: " + str(rc))

    def get_main_data(self, data):
        self.serit_secim = data.serit_secim
        self.direksiyon = data.direksiyon
        self.gaz = data.gaz
        self.fren = data.fren
        self.anlik_hiz = data.anlik_hiz

        self.guc = data.guc
        self.el_freni = data.el_freni

    def get_joy_input_hz(self, data):
        self.joy_hz = data.data

    def get_svl_lane_img_input_hz(self, data):
        self.lane_img_input_hz = data.data

    def get_svl_obj_img_input_hz(self, data):
        self.obj_img_input_hz = data.data

    def get_velocity_input_hz(self, data):
        self.velocity_input_hz = data.data

    def get_io_output_hz(self, data):
        self.io_output_hz = data.data

    def get_serit_takibi_hz(self, data):
        self.serit_takibi_hz = data.data
    
    def get_serit_inference_time(self, data):
        self.serit_inference_time = data.data

    def get_nesne_tespiti_hz(self, data):
        self.nesne_tespiti_hz = data.data

    def get_nesne_inference_time(self, data):
        self.nesne_inference_time = data.data

    def get_hedef_hiz(self, data):
        self.hedef_hiz = data.data

    def pub(self, topic, data):
        res = self.client.publish(topic, data)
        if res[0] != 0:
            self.get_logger().info("problem on publishing " + str(topic) + ", error code:")
            for r in res:
                self.get_logger().info(str(r))

    def output(self):
        data  = '{"secim":' + str(self.serit_secim) + ', "guc":' + str(int(self.guc)) + ', "el_freni":' + str(int(self.el_freni)) + '}'
        self.pub("sge", data)

        data  = '{"direksiyon":' + str(self.direksiyon) + ', "gaz":' + str(int(self.gaz)) + ', "fren":' + str(int(self.fren)) + '}'
        self.pub("dfg", data)

        data  = '{"anlik":' + str(self.anlik_hiz) + ', "hedef":' + str(int(self.hedef_hiz)) + '}'
        self.pub("hiz", data)

        data = [self.joy_hz, self.lane_img_input_hz, self.obj_img_input_hz, self.velocity_input_hz, self.io_output_hz, self.serit_takibi_hz, self.nesne_tespiti_hz]
        data  = '{"joy":' + str(self.joy_hz) + ', "lane_img_input":' + str(int(self.lane_img_input_hz)) + ', "obj_img_input":' + str(int(self.obj_img_input_hz)) + ', '
        data  = '"velocity_input":' + str(self.velocity_input_hz) + ', "io_output":' + str(int(self.io_output_hz)) + ', "serit_takibi":' + str(int(self.serit_takibi_hz)) + ', '
        data  = '"nesne_tespiti":' + str(self.nesne_tespiti_hz) + '}'
        self.pub("hz", data)

def main(args=None):
    rclpy.init(args=args)
    out = Mqtt_output()
    rate = out.create_rate(15)
    while rclpy.ok():
        try:
            rclpy.spin_once(out)
            out.output()
        except KeyboardInterrupt:
            print("Bye Bye!")

    rclpy.shutdown()

if __name__ == "__main__":
    main(args)