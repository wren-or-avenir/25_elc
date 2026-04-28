#多线程版
import queue
import threading

import models.cam as Camera
import models.detector as Detector
import models.stepper as Stepper
import models.tracker as Tracker
import time
import cv2
import Hobot.GPIO as GPIO
import models.status as GPIN
import models.pid as pid

# 新增：线程通信与控制组件
position_queue = queue.Queue(maxsize=1)  # 仅存最新一帧识别结果，丢弃旧数据
dt_queue = queue.Queue(maxsize=1)        # 传递帧间隔时间
running = threading.Event()              # 全局运行信号（代替 break）

cam = Camera.Camera(index = 2, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5, use_kf = True)
stepper_yaw = Stepper.EmmMotor(port ='COM20', baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.EmmMotor(port ='COM7', baudrate = 115200, timeout = 1, motor_id = 2)
heart_beat = GPIN.GPIN(pin=13, mode=1) #呼吸灯，用于表示主程序还在跑
lazer = GPIN.GPIN(pin=16, mode=1)
pid_yaw = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)

