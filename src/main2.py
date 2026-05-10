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

# ---------放在最前面：特别注意的接口和开关----------
camera_index = 0        # 摄像头索引，需根据实际情况调整
yaw_port = '/dev/ttyACM0'      # yaw轴电机串口
pitch_port = '/dev/ttyACM1'     # pitch轴电机串口

use_kf = True           # 是否启用卡尔曼滤波
show_windows = True     # 是否显示调试窗口
# ------------------------------------------------------------------------------

cam = Camera.Camera(index = camera_index, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5, use_kf = use_kf)
stepper_yaw = Stepper.EmmMotor(port = yaw_port, baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.EmmMotor(port = pitch_port, baudrate = 115200, timeout = 1, motor_id = 2)
heart_beat = GPIN.GPIN(pin=13, mode=1) #呼吸灯，用于表示主程序还在跑
lazer = GPIN.GPIN(pin=16, mode=1)
pid_yaw = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)

# ================= 线程安全与数据同步 =================
frame_queue = queue.Queue(maxsize=2)  # 图像帧传递队列

param_lock = threading.Lock()         # 控制参数锁
control_params = {'vel_rpm': 3000, 'acc': 100, 'yaw_sentry_speed': 4.5}

result_lock = threading.Lock()        # 跟踪结果锁
tracking_result = {
    'yaw': 0.0, 'pitch': 0.0, 'dist': 0.0,
    'status': Status.LOST, 'laser_pos': None, 'onfire': False
}

stop_event = threading.Event()        # 全局停止信号
# =====================================================

def nothing(x):
    pass
def init_board():
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Result', 540, 180)
    cv2.namedWindow('Controls', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Controls', 0, 0)
    cv2.resizeWindow('Controls', 320, 500)
    cv2.namedWindow('BIN', cv2.WINDOW_FREERATIO)

    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)

    cv2.createTrackbar('yaw_kp', 'Controls', 2, 100, nothing)   
    cv2.createTrackbar('yaw_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('yaw_kd', 'Controls', 1, 100, nothing)  

    cv2.createTrackbar('pitch_kp', 'Controls', 2, 100, nothing)   
    cv2.createTrackbar('pitch_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('pitch_kd', 'Controls', 1, 100, nothing)  

    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)
    cv2.createTrackbar('acc', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)

def update_params():
    #读原始值
    current_thresh = cv2.getTrackbarPos('Threshold', 'Controls')

    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls')/1000
    yaw_ki = cv2.getTrackbarPos('yaw_ki', 'Controls')/1000000
    yaw_kd = cv2.getTrackbarPos('yaw_kd', 'Controls')/1000000

    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls')/1000
    pitch_ki = cv2.getTrackbarPos('pitch_ki', 'Controls')/100000
    pitch_kd = cv2.getTrackbarPos('pitch_kd', 'Controls')/100000

    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')
    
    #赋值给模块
    detector.threshold_value = current_thresh

    pid_yaw.set_Kp(yaw_kp)
    pid_yaw.set_Ki(yaw_ki)
    pid_yaw.set_Kd(yaw_kd)
    
    pid_pitch.set_Kp(pitch_kp)
    pid_pitch.set_Ki(pitch_ki)
    pid_pitch.set_Kd(pitch_kd)

    #返回控制参
    return vel_rpm, acc

# ================= 线程A：跟踪解算 + PID + 电机控制 =================
def thread_a_worker():
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        # 获取最新滑动条控制参数
        with param_lock:
            vel_rpm = control_params['vel_rpm']
            acc = control_params['acc']
            yaw_sentry_speed = control_params['yaw_sentry_speed']

        # 目标检测
        target = detector.detect(frame)
        
        # 滤波跟踪与解算
        yaw, pitch, dist, status, laser_pos = tracker.track(target)
        
        # 同步 raw 帧
        detector.raw = frame
        tracker.raw = frame
        
        # 更新共享状态供主线程显示与线程B读取
        with result_lock:
            tracking_result['yaw'] = yaw
            tracking_result['pitch'] = pitch
            tracking_result['dist'] = dist
            tracking_result['status'] = status
            tracking_result['laser_pos'] = laser_pos
            tracking_result['onfire'] = tracker.onfire

        # 运行电机控制逻辑（完整保留原分支）
        if status in (Status.TRACK, Status.TMP_LOST):
            try:
                correction_yaw = pid_yaw.compute(yaw)
                stepper_yaw.emm_v5_move_to_angle(angle_deg=-correction_yaw, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
            except Exception as e:
                print(f" Yaw 电机指令异常: {e}")
                        
            try:
                correction_pitch = pid_pitch.compute(pitch)
                stepper_pitch.emm_v5_move_to_angle(angle_deg=-correction_pitch, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
            except Exception as e:
                print(f" pitch 电机指令异常: {e}")
                    
        elif status == Status.LOST:
            pid_yaw.reset()
            pid_pitch.reset()
            # 目标丢失时自动巡视
            try:
                dir = 0.0
                if tracker.last_cy_vel != 0:
                    dir = tracker.last_cy_vel / abs(tracker.last_cy_vel)
                move_speed = dir * yaw_sentry_speed * pid_yaw.Kp
                stepper_yaw.emm_v5_move_to_angle(angle_deg=-move_speed, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
            except Exception as e:
                print(f" Yaw 电机指令异常: {e}")
                        
            try:
                stepper_pitch.emm_v5_move_to_angle(angle_deg=0.0, vel_rpm=vel_rpm, acc=acc, abs_mode=True)
            except Exception as e:
                print(f" pitch 电机指令异常: {e}")
# ================= 线程B：独立管理心跳灯与激光开火 =================
def thread_b_worker():
    while not stop_event.is_set():
        # 独立心跳灯闪烁
        heart_beat.flash()
        
        # 读取跟踪器开火状态并控制激光
        with result_lock:
            is_onfire = tracking_result['onfire']
        if not tracker.onfire:
            lazer.set_value(0)
        else:
            lazer.set_value(1)

        # 保持独立循环频率
        time.sleep(0.015)