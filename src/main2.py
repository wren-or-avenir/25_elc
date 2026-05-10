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

# 新增：线程通信与控制组件
position_queue = queue.Queue(maxsize=1)  # 仅存最新一帧识别结果，丢弃旧数据
dt_queue = queue.Queue(maxsize=1)        # 传递帧间隔时间
running = threading.Event()              # 全局运行信号
angle_q = queue.Queue(maxsize=1)

cam = Camera.Camera(index = camera_index, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5, use_kf = use_kf)
stepper_yaw = Stepper.EmmMotor(port = yaw_port, baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.EmmMotor(port = pitch_port, baudrate = 115200, timeout = 1, motor_id = 2)
heart_beat = GPIN.GPIN(pin=13, mode=1) #呼吸灯，用于表示主程序还在跑
lazer = GPIN.GPIN(pin=16, mode=1)
pid_yaw = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)

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

#线程A ：跟踪解算 + PID + 电机控制
def tracking_and_motor_thread(tracker, yaw_motor, pitch_motor, p_y, p_p, pos_q, dt_q, run_event):
    '''
    独立消费视觉结果
    计算PID
    驱动电机
    '''
    while run_event.is_set():
        try:
            #同时检查“目标位置队列”和“时间差队列”是否有数据。双队列非空时，成对取出。
            if not pos_q.empty() and not dt_q.empty():
                target = pos_q.get()
                dt = dt_q.get()

                res = tracker.track(target)
                yaw, pitch, dist, status, laser_pos = res

                if status in (Tracker.Status.TRACK, Tracker.Status.TMP_LOST):
                    c_yaw = p_y.compute(yaw)#经过pid后的yaw
                    c_pitch = p_p.compute(pitch)#经过pid后的pitch

                    vel_rpm, acc = update_params()

                    if status in (Tracker.Status.TRACK, Tracker.Status.TMP_LOST):#能识别+预测
                        try:
                            print(f"yaw: {yaw}")
                            stepper_yaw.emm_v5_move_to_angle(
                                angle_deg= -c_yaw, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                        except Exception as e:
                            print(f"[电机线程] Yaw 指令异常: {e}")

                        try:
                            print(f"pitch: {pitch}")
                            correction_pitch = pid_pitch.compute(pitch)
                            stepper_yaw.emm_v5_move_to_angle(
                                angle_deg= -c_pitch, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                        except Exception as e:
                            print(f"[电机线程] Pitch 指令异常: {e}")      
                    
                        
                elif status == Tracker.Status.LOST:
                    p_y.reset()
                    p_p.reset()

        except Exception as e:
           print(f"[电机线程] 运行错误: {e}") 
        time.sleep(0.001)
#线程B：独立管理心跳灯与激光开火
def decision_thread(tracker, heart_beat, lazer, angle_q, run_event):
    while run_event.is_set():
        try:
            heart_beat.flash()  # 稳定闪烁

            # 获取最新角度，开火判断逻辑
            if not angle_q.empty():
                pitch, yaw = angle_q.get()
                arrived = tracker.check_onfire(pitch, yaw)
            else:
                arrived = False  # 无数据时默认不触发

            arrived = tracker.check_onfire(pitch, yaw)
            if  arrived:
                lazer.set_value(0)
            else:
                lazer.set_value(1)
        except Exception as e:
            print(f"[决策线程] 运行错误: {str(e)}")
        # 退出时安全关闭 GPIO
        finally:
            heart_beat.set_value(0)
            lazer.set_value(1)