#!/usr/bin/python3
# -*- coding: utf-8 -*-

from dolasim_msgs.msg import IcVeriDolasim
from std_msgs.msg import Bool, Float32, String
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import cv2, time
import rclpy

class mapping(Node):
    def __init__(self):
        super().__init__("Mapping")
        self.sub1 = self.create_subscription(IcVeriDolasim, "/v1/main", self.get_main_data, 10)
        self.sub2 = self.create_subscription(String, "/v1/yonelim", self.get_yonelim_data, 10)
        self.main = IcVeriDolasim()

        self.pub1 = self.create_publisher(IcVeriDolasim, "/v2/main", 10)
        self.pub2 = self.create_publisher(Float32, "/hz_info/haritalama", 10)

        self.main_output_old_time = time.time()
        self.main_output_hz = Float32()

        self.durak1 = 19
        self.durak1_reached = False
        self.durak2 = 20
        self.durak2_reached = False
        self.cikis = 17

        self.graph = nx.DiGraph()

        self.konumlar = [0, 1, 2, 3, 4,
                            5, 6, 7, 8,
                            9, 10, 11, 12,
                            13, 14, 15, 16,
                            17, 18, 19, 20]

        self.speed = {"0-1":0.25,   "1-2":0.25,   "1-5":0.25,   "2-3":0.25,   "2-6":0.25,   "3-4":0.25,   "3-7":0.25,   "4-8":0.25,
                      "5-6":0.25,   "5-18":0.25,  "6-7":0.25,   "6-19":0.25,  "7-8":0.25,   "7-11":0.25,  "8-20":0.25,  "12-20":0.25,
                      "9-18":0.25,  "9-10":0.25,  "9-13":0.25,  "10-11":0.25, "10-18":0.25, "10-14":0.25, "11-12":0.25, "11-15":0.25,
                      "12-16":0.25, "13-14":0.25, "14-15":0.25, "15-16":0.25, "17-18":0.25}

        self.graph.add_nodes_from(self.konumlar)
        self.add_edges()

        self.lines = [(1260,700), (1260, 530), (1260, 360), (1260, 190), (1260, 20),
                                 (850, 530), (850, 360), (850, 190), (850, 20),
                                 (440, 530), (440, 360), (440, 190), (440, 20),
                                 (30, 530), (30, 360), (30, 190), (30, 20),
                                 (645, 700), (645, 530), (645, 360), (500, 20)]

        self.base_img = self.get_base_img()
        self.map_img = None

        self.before = None
        self.now = 0
        self.window_position = self.lines[0]
        self.yonelim = "ileri"
    
    def get_main_data(self, data):
        self.main = data

    def get_yonelim_data(self, data):
        self.yonelim = data.data
        self.map_img = self.runner(self.yonelim)

    def get_compressed_image(self, data):
        try:
            img = self.cvb.cv2_to_compressed_imgmsg(data, "jpeg")
            return img
        except Exception as e:
            print("mapping compression error", e)

    def get_base_img(self):
        img = np.zeros((720, 1280, 3), dtype = "uint8")
        start = self.lines[0]
        end = self.lines[1]
        img = cv2.line(img, (start[0]+5, start[1]+5), (end[0]+5, end[1]+5), (255, 255, 255), 2)
        img = cv2.line(img, (start[0]-5, start[1]-5), (end[0]-5, end[1]-5), (255, 255, 255), 2)
        start = self.lines[18]
        end = self.lines[17]
        img = cv2.line(img, start, end, (255, 255, 255), 2)

        for i in range(1, 10, 4):
            img = self.draw_vertical_line(img, i)
            img = self.draw_horizontal_line(img, i)

        img = self.draw_vertical_line(img, 13)

        for i in self.lines:
            img = cv2.circle(img, i, 15, (255, 255, 255), -1)

        img = cv2.circle(img, self.lines[10], 50, (0, 0, 0), -1)
        img = cv2.circle(img, self.lines[10], 50, (255, 255, 255), 2)   

        return img

    def draw_horizontal_line(self, img, ind):
        for j in range(ind, ind+4):
            start = self.lines[j]
            end = self.lines[j + 4]
            img = cv2.line(img, (start[0]-5, start[1]-5), (end[0]-5, end[1]-5), (255, 255, 255), 2)
            img = cv2.line(img, (start[0]+5, start[1]+5), (end[0]+5, end[1]+5), (255, 255, 255), 2)

        return img

    def draw_vertical_line(self, img, ind):
        for j in range(ind, ind+3):
            start = self.lines[j]
            end = self.lines[j + 1]
            img = cv2.line(img, (start[0]-5, start[1]-5), (end[0]-5, end[1]-5), (255, 255, 255), 2)
            img = cv2.line(img, (start[0]+5, start[1]+5), (end[0]+5, end[1]+5), (255, 255, 255), 2)

        return img

    def color_line(self, img, s, e, inc = 5):
        img = cv2.circle(img, s, 15, (0, 0, 255), -1)
        img = cv2.circle(img, e, 15, (0, 0, 255), -1)
        img = cv2.line(img, (s[0]+inc, s[1]+inc), (e[0]+inc, e[1]+inc), (0, 0, 255), 2)

        return img

    def color_arc(self, img, s, e, n, b = False):
        img = cv2.circle(img, self.lines[10], 50, (0, 0, 0), -1)
        img = cv2.circle(img, self.lines[10], 50, (255, 255, 255), 2)
        
        rad = 50
        cen = self.lines[10]

        if not b:
            if s == 19:
                if n == 11:
                    start = 270
                    end = 360

                elif n == 14:
                    start = 180
                    end = 360

                elif n == 9:
                    start = 90
                    end = 360

            elif s == 11:
                if n == 19:
                    start = 0
                    end = 270

                elif n == 14:
                    start = 180
                    end = 270

                elif n == 9:
                    start = 90
                    end = 270

            elif s == 14:
                if n == 11:
                    start = 180
                    end = -90

                if n == 19:
                    start = 0
                    end = 180

                if n == 9:
                    start = 90
                    end = 180

            elif s == 9:
                if n == 11:
                    start = 90
                    end = -90

                elif n == 19:
                    start = 0
                    end = 90

                elif n == 14:
                    start = 90
                    end = -180

        else:
            if e == 19:
                if n == 11:
                    start = 180
                    end = 270

                elif n == 14:
                    start = 180
                    end = 360

                elif n == 9:
                    start = 90
                    end = 360

            elif e == 11:
                if n == 19:
                    start = 270
                    end = 360

                elif n == 14:
                    start = 180
                    end = 270

                elif n == 9:
                    start = 90
                    end = 270

            elif e == 14:
                if n == 11:
                    start = 180
                    end = -90

                if n == 19:
                    start = 0
                    end = 180

                if n == 9:
                    start = 90
                    end = 180

            elif e == 9:
                if n == 11:
                    start = 90
                    end = -90

                elif n == 19:
                    start = 0
                    end = 90

                elif n == 14:
                    start = 90
                    end = -180

        cv2.ellipse(img, cen, (rad,rad), 0, start, end, (0,0,255), 2)
        return img

    def add_edges(self):
        self.graph.add_edges_from([(0,1), (1, 0)])

        for i in range(1, 14, 4):
            for j in range(i, i+3):
                self.graph.add_edge(j, j+1)
                self.graph.add_edge(j+1, j)

        for i in range(1, 10, 4):
            for j in range(i, i+4):
                self.graph.add_edge(j, j+4)
                self.graph.add_edge(j+4, j)

        self.graph.remove_edges_from([(5, 9), (9, 5)])
        self.graph.remove_edges_from([(6, 10), (10, 6)])
        self.graph.remove_edges_from([(8, 12), (12, 8)])

        self.graph.add_edges_from([(5, 18), (18, 5)])
        self.graph.add_edges_from([(9, 18), (18, 9)])

        self.graph.add_edges_from([(6, 19), (19, 6)])
        self.graph.add_edges_from([(10, 19), (19, 10)])

        self.graph.add_edges_from([(8, 20), (20, 8)])
        self.graph.add_edges_from([(12, 20), (20, 12)])

        self.graph.add_edge(18, 17)

    def olasi_yonler(self, before, now, nesne):
        ols = list()
        if now >= 17:
            if now == 17:
                ols.append(["asagi", -1])

            elif now == 18:
                if self.durak1_reached and self.durak2_reached:
                    ols.append(["asagi", 17])

                else:
                    if before == 9:
                        ols.append(["ileri", 5])

                    elif before == 5:
                        ols.append(["ileri", 9])

            elif now == 19:
                if before == 6:
                    ols.append(["ileri", 10])

                elif before == 10:
                    ols.append(["ileri", 6])

            elif now == 20:
                if before == 12:
                    ols.append(["ileri", 8])

                elif before == 8:
                    ols.append(["ileri", 12])
            
        else:
            for node in self.graph.neighbors(now):
                if node == 18:
                    if now == 5:
                        node = 9

                    else:
                        node = 5

                elif node == 19:
                    if node == 6:
                        node = 10

                    else:
                        node = 6

                elif node == 20:
                    if now == 12:
                        node = 8

                    else:
                        node = 12

                print(node)
                if node - now == 1:
                    if before  - now == 4:
                        ols.append(["sola", node])

                    elif before - now == -4:
                        ols.append(["saga", node])

                    elif before - now == -1:
                        ols.append(["ileri", node])

                elif node - now == -1:
                    if before - now == 4:
                        ols.append(["saga", node])

                    elif before - now == -4:
                        ols.append(["sola", node])

                    elif before - now == 1:
                        ols.append(["ileri", node])

                elif node - now == 4:
                    if before - now == 1:
                        ols.append(["saga", node])

                    elif before - now == -1:
                        ols.append(["sola", node])

                    elif before - now == -4:
                        ols.append(["ileri", node])

                elif node - now == -4:
                    if before - now == 1:
                        ols.append(["saga", node])

                    elif before - now == -1:
                        ols.append(["sola", node])

                    elif before - now == 4:
                        ols.append(["ileri", node])

        return ols

    def runner(self, yonelim):
        if self.now == 19:
            self.durak1_reached = True

        if self.now == 20:
            self.durak2_reached = True

        d1_path = nx.shortest_path(self.graph, source=self.now, target=self.durak1)
        d1_path.pop(d1_path.index(self.now))
        d2_path = nx.shortest_path(self.graph, source=self.now, target=self.durak2)
        d2_path.pop(d2_path.index(self.now))

        cikis_path = nx.shortest_path(self.graph, source=self.now, target=self.cikis)
        cikis_path.pop(cikis_path.index(self.now))

        if (not self.durak1_reached) and (not self.durak2_reached):
            if len(d1_path) < len(d2_path):
                shortest_path = d1_path

            else:
                shortest_path = d2_path

        elif self.durak1_reached and not self.durak2_reached:
            shortest_path = d2_path

        elif not self.durak1_reached and self.durak2_reached:
            shortest_path = d1_path

        else:
            shortest_path = cikis_path

        hareket_yonu = shortest_path[0]

        if hareket_yonu == self.durak1: self.durak1_reached = True
        if hareket_yonu == self.durak2: self.durak2_reached = True

        if hareket_yonu != self.cikis:
            if self.before == None:
                self.graph.remove_edge(hareket_yonu, self.now)

            else:
                self.graph.add_edge(self.before[0], self.before[1])
                self.graph.remove_edge(hareket_yonu, self.now)

        img = self.base_img.copy()

        start = self.lines[self.now]
        end = self.lines[hareket_yonu]
        if hareket_yonu == 10:
            img = self.color_line(img, start, end)
            self.color_arc(img, self.now, hareket_yonu, shortest_path[1])
            #self.color_arc(img, 9, hareket_yonu, 14)

        elif self.now == 10:
            img = self.color_line(img, start, end)
            self.color_arc(img, self.now, hareket_yonu, self.before[1], True)
            #self.color_arc(img, self.now, 9, 14, True)

        else:
            if abs(self.now - hareket_yonu) == 1:
                img = self.color_line(img, start, end, 5)

            else:
                img = self.color_line(img, start, end, -5)
            
        cv2.imshow("img", img)
        if cv2.waitKey(25) % 0xFF == ord('q'):
            pass

        self.before = [hareket_yonu, self.now]
        self.now = hareket_yonu
        self.main.map_img = self.get_compressed_image(img)

    def output(self):
        self.main_output_new_time = time.time()
        hz = 1.0 / (self.main_output_new_time - self.main_output_old_time)
        self.main_output_old_time = self.main_output_new_time
        self.main_output_hz.data = hz

        self.pub1.publish(self.main)
        self.pub2.publish(self.main_output_hz)


def main(args=None):
    rclpy.init(args=args)
    harita = mapping()
    rate = harita.create_rate(30)

    while rclpy.ok():
        try:
            rclpy.spin_once(harita)
            harita.output()

        except KeyboardInterrupt:
            print("good bye")

    rclpy.shutdown()

if __name__ == "__main__":
    main(args)