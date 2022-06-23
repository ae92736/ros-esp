#!/usr/bin/python3
# -*- coding: utf-8 -*-

from std_msgs.msg import Bool, Float32, Int8
from sensor_msgs.msg import CompressedImage, Imu, Image
from dolasim_msgs.msg import IcVeriDolasim
from cv_bridge import CvBridge
from .robotaksi_ui import *
from qt_material import *
from Custom_Widgets.Widgets import *
from rclpy.node import Node
import numpy as np
import os, sys, PySide2extn
import rclpy, time, cv2

class signals(QObject):
    info = Signal(list)
    kontrol = Signal(list)
    secim = Signal(int)
    lane_image = Signal(np.ndarray)
    obj_image = Signal(np.ndarray)

class worker(QRunnable):
    def __init__(self):
        super(worker, self).__init__()
        self.node = Node("robotaksi_gui")
        self.node.__init__("gui")
        self.cvb = CvBridge()

        self.sub1 = self.node.create_subscription(IcVeriDolasim, "/v3/main", self.get_input_main, 10)

        self.sub2 = self.node.create_subscription(Float32, "/hz_info/joy", self.get_joy_input_hz, 10)
        self.sub3 = self.node.create_subscription(Float32, "/hz_info/svl_lane_img_input", self.get_svl_lane_img_input_hz, 10)
        self.sub4 = self.node.create_subscription(Float32, "/hz_info/svl_obj_img_input", self.get_svl_obj_img_input_hz, 10)
        self.sub5 = self.node.create_subscription(Float32, "/hz_info/svl_vel_input", self.get_svl_vel_input_hz, 10)
        self.sub6 = self.node.create_subscription(Float32, "/hz_info/output_hz", self.get_io_output_hz, 10)
        self.sub7 = self.node.create_subscription(Float32, "/hz_info/serit_takibi", self.get_serit_takibi_hz, 10)
        self.sub8 = self.node.create_subscription(Float32, "/hz_info/cnn_inferance_time", self.get_serit_inferance_hz, 10)
        self.sub9 = self.node.create_subscription(Float32, "/hz_info/nesne_tespiti", self.get_nesne_tespiti_hz, 10)
        self.sub10 = self.node.create_subscription(Float32, "/hz_info/yolo_inferance_time", self.get_nesne_inferance_time, 10)
        
        self.pub1 = self.node.create_publisher(Int8, "/feedback/serit_secim", 10)
        self.pub2 = self.node.create_publisher(Float32, "/feedback/hedef_hiz", 10)
        self.pub3 = self.node.create_publisher(Int8, "/feedback/guc", 10)
        self.pub4 = self.node.create_publisher(Int8, "/feedback/el_freni", 10)
        self.pub5 = self.node.create_publisher(Bool, "/feedback/logger", 10)
        self.pub6 = self.node.create_publisher(Bool, "/feedback/serit_takibi", 10)
        self.pub7 = self.node.create_publisher(Bool, "/feedback/nesne_tespiti", 10)
        self.pub8 = self.node.create_publisher(Bool, "/feedback/joy_enable", 10)

        self.joy_input_hz = 0
        self.svl_lane_img_output_hz = 0
        self.svl_obj_img_output_hz = 0
        self.svl_vel_output_hz = 0
        self.io_output_hz = 0
        self.serit_output_hz = 0
        self.serit_inferance_hz = 0
        self.nesne_output_hz = 0
        self.nesne_inferance_hz = 0
        
        self.sig = signals()
        self.info = [None, None, None, None] * 5
        self.kontrol_data = [0, 0, 0, 0]
        self.input_secim = 0
        self.lane_image = np.ndarray((1280,720,3))
        self.obj_image = np.ndarray((1280,720,3))
        self.close_event = False

    def run(self):
        while rclpy.ok() and not self.close_event:
            rclpy.spin_once(self.node)
            time.sleep(0.01)
        
        rclpy.shutdown()

    def get_joy_input_hz(self, data):
        self.joy_output_hz = data.data

    def get_svl_lane_img_input_hz(self, data):
        self.svl_lane_img_output_hz = data.data

    def get_svl_obj_img_input_hz(self, data):
        self.svl_obj_img_output_hz = data.data

    def get_svl_vel_input_hz(self, data):
        self.svl_vel_output_hz = data.data

    def get_io_output_hz(self, data):
        self.io_output_hz = data.data

    def get_serit_takibi_hz(self, data):
        self.serit_output_hz = data.data

    def get_serit_inferance_hz(self, data):
        self.serit_inferance_hz = data.data

    def get_nesne_tespiti_hz(self, data):
        self.nesne_output_hz = data.data

    def get_nesne_inferance_time(self, data):
        self.nesne_inferance_hz = data.data

    def get_input_lane_img(self, data):
        self.lane_image = self.cvb.compressed_imgmsg_to_cv2(data)
        self.lane_image = cv2.cvtColor(self.lane_image, cv2.COLOR_BGR2RGB)

    def get_input_obj_img(self, data):
        self.obj_image = self.cvb.compressed_imgmsg_to_cv2(data)
        self.obj_image = cv2.cvtColor(self.obj_image, cv2.COLOR_BGR2RGB)

    def get_input_main(self, data):
        self.get_input_lane_img(data.lane_img)
        self.get_input_obj_img(data.obj_img)
        self.input_secim = data.serit_secim
        self.input_vites = data.vites
        self.input_log_toggle = data.log_enabled
        self.kontrol_data[0] = data.direksiyon * 100
        self.kontrol_data[1] = data.gaz
        self.kontrol_data[2] = data.fren
        self.kontrol_data[3] = data.anlik_hiz
        self.input_guc = data.guc
        self.input_el_freni = data.el_freni
        self.main_updater()

    def main_updater(self):
        self.info[0] = ["/joy", "Joy", "joy_node", str(self.joy_input_hz)]
        self.info[1] = ["/lgsvl/lane_img_output", "CompressedImg", "svl", str(self.svl_lane_img_output_hz)]
        self.info[2] = ["/lgsvl/obj_img_output", "CompressedImg", "svl", str(self.svl_obj_img_output_hz)]
        self.info[3] = ["/lgsvl/vel_output", "VehicleOdometry", "svl", str(self.svl_vel_output_hz)]
        self.info[4] = ["/v1/main", "IcVeriDolasim", "serit_takibi", str(self.serit_output_hz)]
        self.info[5] = ["cnn time", "inferance", "serit_takibi", str(self.serit_inferance_hz)]
        self.info[6] = ["/v2/main", "IcVeriDolasim", "nesne_tespiti", str(self.nesne_output_hz)]
        self.info[7] = ["/nesne time", "inferance", "nesne_tespiti", str(self.nesne_inferance_hz)]
        self.info[8] = ["/control_input", "VehicleControl", "output", str(self.io_output_hz)]

        self.sig.info.emit(self.info)
        self.sig.kontrol.emit(self.kontrol_data)
        self.sig.secim.emit(self.input_secim)
        self.sig.lane_image.emit(self.lane_image)
        self.sig.obj_image.emit(self.obj_image)

class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.shadow = QGraphicsDropShadowEffect(self)
        self.shadow.setBlurRadius(50)
        self.shadow.setXOffset(0)
        self.shadow.setYOffset(0)
        self.shadow.setColor(QColor(0, 92, 157, 550))
        self.ui.centralwidget.setGraphicsEffect(self.shadow)

        self.setWindowIcon(QtGui.QIcon(":/icons/icons/feather/airplay.svg"))
        self.setWindowTitle("Mekatek Robotaksi Manegement System")
        QSizeGrip(self.ui.size_grip)

        self.ui.minimize_window_button.clicked.connect(lambda: self.showMinimized())
        self.ui.close_window_button.clicked.connect(lambda: self.close_gui())
        self.ui.restore_window_button.clicked.connect(lambda: self.restore_or_maximize_window())
        self.ui.lane_button.clicked.connect(lambda: self.ui.pages.setCurrentWidget(self.ui.lane_page))
        self.ui.object_button.clicked.connect(lambda: self.ui.pages.setCurrentWidget(self.ui.object_page))
        self.ui.info_button.clicked.connect(lambda: self.ui.pages.setCurrentWidget(self.ui.info_page))
        self.ui.control_button.clicked.connect(lambda: self.ui.pages.setCurrentWidget(self.ui.control_page))
        self.ui.menu_button.clicked.connect(lambda: self.slideLeftMenu())
        self.ui.sinir_set_btn.clicked.connect(lambda: self.sinir_set())
        self.ui.kontrol_set_btn.clicked.connect(lambda: self.kontrol_set())
        self.ui.sinir_set_ek.clicked.connect(lambda: self.sinir_set_ek())
        self.ui.kontrol_set_ek.clicked.connect(lambda: self.kontrol_set_ek())

        self.serit_control_item = QTableWidgetItem()
        self.ui.control_info_table.setItem(1, -1, self.serit_control_item)
        self.serit_control_item.setText("Stopped")

        self.nesne_control_item = QTableWidgetItem()
        self.ui.control_info_table.setItem(1, 0, self.nesne_control_item)
        self.nesne_control_item.setText("Stopped")

        self.harita_control_item = QTableWidgetItem()
        self.ui.control_info_table.setItem(1, 1, self.harita_control_item)
        self.harita_control_item.setText("Stopped")

        self.joy_control_item = QTableWidgetItem()
        self.ui.control_info_table.setItem(1, 2, self.joy_control_item)
        self.joy_control_item.setText("Stopped")

        self.logger_control_item = QTableWidgetItem()
        self.ui.control_info_table.setItem(1, 3, self.logger_control_item)
        self.logger_control_item.setText("Stopped")

        self.stype_control_item = QTableWidgetItem()
        self.ui.control_info_table.setItem(1, 4, self.stype_control_item)
        self.stype_control_item.setText("Stopped")

        self.ui.table_serit_set.clicked.connect(lambda: self.serit_set_table())
        self.ui.table_nesne_set.clicked.connect(lambda: self.nesne_set_table())
        self.ui.table_harita_set.clicked.connect(lambda: self.harita_set_table())
        self.ui.table_joy_set.clicked.connect(lambda: self.joy_set_table())
        self.ui.table_logger_set.clicked.connect(lambda: self.logger_set_table())
        self.ui.table_stype_set.clicked.connect(lambda: self.stype_set_table())

        self.ui.header_frame.mouseMoveEvent = self.moveWindow

        loadJsonStyle(self, self.ui, "/home/aye/robotaksi_main/main_ws/rb22_main/rb22_sim_serit/src/gui/resource/style.json")
        self.ui.direksiyon_widget.updateValue(0)
        self.colortp = ((237, 113, 12),(6, 202, 209),(196, 2, 2))
        self.ui.gfh_bar.spb_lineColor(self.colortp)
        self.ui.gfh_bar.spb_lineCap(("RoundCap", "RoundCap", "RoundCap"))
        self.ui.gfh_bar.spb_lineStyle(("SolidLine", "SolidLine", "SolidLine"))
        self.ui.gfh_bar.spb_lineWidth(10)
        self.ui.gfh_bar.variableWidth(True)
        self.ui.gfh_bar.spb_setPathHidden(True)
        self.ui.gfh_bar.spb_setInitialPos(('North', 'West', 'East'))
        self.ui.gfh_bar.spb_setDirection(("Clockwise", "Clockwise", "AntiClockwise"))
        self.ui.gfh_bar.spb_setMinimum((0, 0, 0))
        self.ui.gfh_bar.spb_setMaximum((15, 1, 1))
        self.ui.gfh_bar.spb_setValue((2.5, 0.25, 0.25))

        self.ros = worker()
        self.ros.setAutoDelete(False)

        self.ros.sig.lane_image.connect(self.set_serit_img)
        self.ros.sig.obj_image.connect(self.set_nesne_img)
        self.ros.sig.info.connect(self.set_topic_info_table)
        self.ros.sig.secim.connect(self.set_serit_secim)
        self.ros.sig.kontrol.connect(self.set_kontrol)

        QThreadPool.globalInstance().start(self.ros)

        self.showMaximized()

    @Slot()
    def set_topic_info_table(self, datas):
        for i in range(0, 9):
            data = datas[i]
            for j in range(0, len(data)):
                widget = QTableWidgetItem()
                self.ui.topic_info_table.setItem(i, j, widget)
                item = self.ui.topic_info_table.item(i,j)
                item.setText(str(data[j]))
    
    @Slot(np.ndarray)
    def set_serit_img(self, img):
        pixmap = self.convert_cv_qt(img)
        self.ui.img_label.setPixmap(pixmap)

    @Slot(np.ndarray)
    def set_nesne_img(self, img):
        pixmap = self.convert_cv_qt(img)
        self.ui.obj_label.setPixmap(pixmap)
    
    def convert_cv_qt(self, rgb_image):
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format)

    @Slot()
    def set_serit_secim(self, data):
        self.ui.kontrol_deger.setText(str(data))
        self.ui.serit_ek_2.setText(str(data))

    @Slot()
    def set_kontrol(self, data):
        self.ui.direksiyon_widget.updateValue(data[0])
        self.ui.gfh_bar.spb_setValue((data[3], data[1], data[2]))

    def serit_set_table(self):
        deg = self.serit_control_item.text()
        if deg == "Stopped":
            self.serit_control_item.setText("Running")
            data = Bool()
            data.data = True
            self.ros.pub6.publish(data)

        elif deg == "Running":
            self.serit_control_item.setText("Stopped")
            data = Bool()
            data.data = False
            self.ros.pub6.publish(data)

        else:
            print("there has been a problem in setting the serit value of table")

    def nesne_set_table(self):
        deg = self.nesne_control_item.text()
        if deg == "Stopped":
            self.nesne_control_item.setText("Running")
            data = Bool()
            data.data = True
            self.ros.pub7.publish(data)

        elif deg == "Running":
            self.nesne_control_item.setText("Stopped")
            data = Bool()
            data.data = False
            self.ros.pub7.publish(data)

        else:
            print("there has been a problem in setting the nesne value of table")

    def harita_set_table(self):
        deg = self.harita_control_item.text()
        if deg == "Stopped":
            self.harita_control_item.setText("Running")

        elif deg == "Running":
            self.harita_control_item.setText("Stopped")

        else:
            print("there has been a problem in setting the harita value of table")

    def joy_set_table(self):
        deg = self.joy_control_item.text()
        if deg == "Stopped":
            self.joy_control_item.setText("Running")
            data = Bool()
            data.data = True
            self.ros.pub8.publish(data)

        elif deg == "Running":
            self.joy_control_item.setText("Stopped")
            data = Bool()
            data.data = False
            self.ros.pub8.publish(data)

        else:
            print("there has been a problem in setting the joy value of table")

    def logger_set_table(self):
        deg = self.logger_control_item.text()
        if deg == "Stopped":
            self.logger_control_item.setText("Running")
            data = Bool()
            data.data = True
            self.ros.pub5.publish(data)

        elif deg == "Running":
            self.logger_control_item.setText("Stopped")
            data = Bool()
            data.data = False
            self.ros.pub5.publish(data)

        else:
            print("there has been a problem in setting the logger value of table")

    def stype_set_table(self):
        deg = self.stype_control_item.text()
        if deg == "Stopped":
            self.stype_control_item.setText("Running")

        elif deg == "Running":
            self.stype_control_item.setText("Stopped")

        else:
            print("there has been a problem in setting the stype value of table")

    def sinir_set(self):
        self.hiz_siniri = (self.ui.sinir_slider.value() / 10)
        self.ui.sinir_deger.setText(str(self.hiz_siniri))
        data = Float32()
        data.data = self.hiz_siniri
        self.ros.pub2.publish(data)

    def kontrol_set(self):
        self.serit_secim = int(self.ui.kontrol_slider.value())
        self.ui.kontrol_deger.setText(str(self.serit_secim))
        self.ui.serit_ek_2.setText(str(self.serit_secim))
        data = Int8()
        data.data = self.serit_secim
        self.ros.pub1.publish(data)

    def sinir_set_ek(self):
        deg = self.ui.sinir_ek.text()
        if not deg == '':
            self.hiz_siniri = float(deg)
            self.ui.sinir_deger.setText(str(self.hiz_siniri))
            self.ui.sinir_slider.setValue(int(self.hiz_siniri * 10))
            data = Float32()
            data.data = self.hiz_siniri
            self.ros.pub2.publish(data)

    def kontrol_set_ek(self):
        deg = self.ui.serit_ek.text()
        if not deg == '':
            self.serit_secim = int(deg)
            self.ui.kontrol_deger.setText(str(self.serit_secim))
            self.ui.kontrol_slider.setValue(self.serit_secim)
            self.ui.serit_ek_2.setText(str(self.serit_secim))
            data = Int8()
            data.data = self.serit_secim
            self.ros.pub1.publish(data)

    def restore_or_maximize_window(self):
        if self.isMaximized():
            self.showNormal()
            self.ui.restore_window_button.setIcon(QtGui.QIcon(u":/icons/icons/cil-window-restore.png"))
        else:
            self.showMaximized()
            self.ui.restore_window_button.setIcon(QtGui.QIcon(u":/icons/icons/cil-window-maximize.png"))

    def moveWindow(self, e):
        if not self.isMaximized():
            if e.buttons() == Qt.LeftButton:
                self.move(self.pos() + e.globalPos() - self.clickPosition)
                self.clickPosition = e.globalPos()
                e.accept()

    def mousePressEvent(self, event):
        self.clickPosition = event.globalPos()

    def slideLeftMenu(self):
        width = self.ui.left_menu_cont_frame.width()

        if width == 80:
            newWidth = 275

        else:
            newWidth = 80

        self.animation = QPropertyAnimation(self.ui.left_menu_cont_frame, b"minimumWidth")#Animate minimumWidht
        self.animation.setDuration(250)
        self.animation.setStartValue(width)#Start value is the current menu width
        self.animation.setEndValue(newWidth)#end value is the new menu width
        self.animation.setEasingCurve(QtCore.QEasingCurve.InOutQuart)
        self.animation.start()

    def close_gui(self):
        self.ros.close_event = True
        time.sleep(0.03)
        self.close()

def main(args=None):
    time.sleep(10)
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
    rclp.shutdown()
    
if __name__ == "__main__":
    main(args)    