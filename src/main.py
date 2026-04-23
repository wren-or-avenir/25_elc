import model.cam as Camera
import model.detector as Detector
import model.stepper as Stepper
import model.tracker as Tracker
import time
import cv2
#import Hobot.GPIO as GPIO
#import model.status as GPIN
import model.pid as pid
from model.tracker import Tracker, Status

cam = Camera.Camera(index = 4, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5, use_kf = True)
stepper_yaw = Stepper.EmmMotor(port ='COM20', baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.EmmMotor(port ='COM7', baudrate = 115200, timeout = 1, motor_id = 2)
#heart_beat = GPIN.GPIN(pin=13, mode=1) #呼吸灯，用于表示主程序还在跑
#lazer = GPIN(pin=16, mode=1)
pid_yaw = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)

def nothing(x):
    pass

def init_board():

def update_hsv():

def main ():