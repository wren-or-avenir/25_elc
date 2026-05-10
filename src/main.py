#单线程版
import cv2
import time
from models.cam import Camera
from models.detector import Detector
from models.tracker import Tracker, Status
from models.stepper import SysParams, EmmMotor
from models.pid import PIDController
import Hobot.GPIO as GPIO
from models.status import GPIN
from models.dm_imu import IMU

# ---------放在最前面：特别注意的接口和开关----------
camera_index = 0        # 摄像头索引，需根据实际情况调整
yaw_port = '/dev/ttyACM0'      # yaw轴电机串口
pitch_port = '/dev/ttyACM1'     # pitch轴电机串口

use_kf = True           # 是否启用卡尔曼滤波
show_windows = True     # 是否显示调试窗口
# ------------------------------------------------------------------------------

# -----------------模块初始化--------------------
camera = Camera(index = camera_index, width = 640, height = 480)
detector = Detector(min_area = 5000, max_area = 500000)
tracker = Tracker(f_pixel_h = 725.6, real_height = 17.5, use_kf = use_kf) 

stepper_yaw = EmmMotor(port = yaw_port, baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = EmmMotor(port = pitch_port, baudrate = 115200, timeout = 1, motor_id = 2)
pid_yaw = PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
lazer = GPIN(pin=16, mode=1) #激光笔控制
heart_beat = GPIN(pin=18, mode=1) #呼吸灯，用于表示主程序还在跑
# -------------------------------------------------------------------------------

def nothing(x):
    pass
def init_board():
    """初始化调试窗口和滑动条"""
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 300, 150)
    cv2.namedWindow('DETECTOR', cv2.WINDOW_FREERATIO)  
    cv2.namedWindow('BIN', cv2.WINDOW_FREERATIO)      

    cv2.createTrackbar('system_delay', 'Controls', 21, 100, nothing)

    cv2.createTrackbar('yaw_kp', 'Controls', 28, 100, nothing)   
    cv2.createTrackbar('yaw_ki', 'Controls', 2, 100, nothing)   
    cv2.createTrackbar('yaw_kd', 'Controls', 20, 100, nothing)

    cv2.createTrackbar('yaw_sentry_speed', 'Controls', 45, 100, nothing)

    cv2.createTrackbar('pitch_kp', 'Controls', 22, 100, nothing)   
    cv2.createTrackbar('pitch_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('pitch_kd', 'Controls', 16, 100, nothing)  

    cv2.createTrackbar('onfire_tol', 'Controls', 5, 100, nothing) # 开火容忍度，单位为0.1度

    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)
    cv2.createTrackbar('acc', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)

def update_params():
    """回调获取滑块参数"""
    system_delay = cv2.getTrackbarPos('system_delay', 'Controls')/10
    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls')/1000
    yaw_ki = cv2.getTrackbarPos('yaw_ki', 'Controls')/10000
    yaw_kd = cv2.getTrackbarPos('yaw_kd', 'Controls')/10000

    yaw_sentry_speed = cv2.getTrackbarPos('yaw_sentry_speed', 'Controls')/10

    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls')/1000
    pitch_ki = cv2.getTrackbarPos('pitch_ki', 'Controls')/10000
    pitch_kd = cv2.getTrackbarPos('pitch_kd', 'Controls')/10000

    onfire_tol = cv2.getTrackbarPos('onfire_tol', 'Controls')/10

    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')

    tracker.onfire_tol = onfire_tol
    tracker.system_delay = system_delay
    #赋值给模块
    pid_yaw.set_Kp(yaw_kp)
    pid_yaw.set_Ki(yaw_ki)
    pid_yaw.set_Kd(yaw_kd)
    
    pid_pitch.set_Kp(pitch_kp)
    pid_pitch.set_Ki(pitch_ki)
    pid_pitch.set_Kd(pitch_kd)

    return vel_rpm, acc, yaw_sentry_speed


def main ():
    init_board()
    if not camera.isOpened():
        print("摄像头打不开")
        return
    prev_time = time.time()
    print(" 系统启动，按 'q' 退出...")

    try:
        while True:
            # # 呼吸灯，证明主程序在运行(单线程中闪烁频率完全受制于主循环的运行速度)
            heart_beat.flash()

            #读帧
            ret, frame = camera.read()
            if not ret: 
                print("无法读帧")
                break

            # 计算 FPS
            curr_time = time.time()
            fps = 1.0 / max(curr_time - prev_time, 1e-6)
            prev_time = curr_time

            #更新参数
            vel_rpm, acc, yaw_sentry_speed = update_params()

            #识别目标
            target = detector.detect(frame)

            # 滤波跟踪与解算
            res = tracker.track(target)
            yaw, pitch, dist, status, laser_pos = res

            #激光开火判断
            if not tracker.onfire:
                lazer.set_value(0)
            else:
                lazer.set_value(1)

            # 格式化状态文本
            if status == Status.TRACK:
                info = f"[TRACK] Yaw:{yaw:.2f} Pitch:{pitch:.2f} Dist:{dist:.1f}cm"
            elif status == Status.TMP_LOST:
                info = f"[PREDICT] Predicting... Dist:{dist:.1f}cm"
            else:
                info = "[LOST] Searching..."
            
            # 终端打印 FPS + 状态
            print(f"FPS: {fps:.1f} | {info}")

            # 同步 raw 帧
            detector.raw = frame
            tracker.raw = frame
            
            # 绘制与展示
            if show_windows:
                # 获取绘制结果
                vis_det, bin_img = detector.display(dis=1)
                vis_trk = tracker.display(dis=1, laser_pos=laser_pos)
                
                # 窗口显示
                if vis_det is not None:
                    cv2.imshow("DETECTOR", vis_det)
                if bin_img is not None:
                    cv2.imshow("BIN", bin_img)
                if vis_trk is not None:
                    cv2.imshow("Tracker", vis_trk)
            else:
                cv2.destroyAllWindows()

            # 运行电机
            if status in (Status.TRACK, Status.TMP_LOST):#能识别+预测
                try:
                    print(f"yaw: {yaw}")
                    correction_yaw = pid_yaw.compute(yaw)#经过pid后的yaw
                    stepper_yaw.emm_v5_move_to_angle(
                        angle_deg= -correction_yaw, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                except Exception as e:
                    print(f" Yaw 电机指令异常: {e}")
                            
                try:
                    print(f"pitch: {pitch}")
                    correction_pitch = pid_pitch.compute(pitch)#经过pid后的pitch
                    stepper_pitch.emm_v5_move_to_angle(
                        angle_deg= -correction_pitch, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                except Exception as e:
                    print(f" pitch 电机指令异常: {e}")
                
            elif status == Status.LOST:#丢帧超多阈值，停止运动
                #重置pid
                pid_yaw.reset()
                pid_pitch.reset()
                # 目标丢失时自动巡视
                try:
                    dir = 0.0
                    if tracker.last_cy_vel != 0:
                        dir = tracker.last_cy_vel / abs(tracker.last_cy_vel)
                    print(f"last_cy_vel: {tracker.last_cy_vel} | dir {dir}")
                    move_speed = dir * yaw_sentry_speed * pid_yaw.Kp
                    stepper_yaw.emm_v5_move_to_angle(
                        angle_deg=-move_speed, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                except Exception as e:
                    print(f" Yaw 电机指令异常: {e}")
                            
                try:
                    stepper_pitch.emm_v5_move_to_angle(
                        angle_deg= 0.0, vel_rpm=vel_rpm, acc=acc, abs_mode=True)
                except Exception as e:
                    print(f" pitch 电机指令异常: {e}")
            # 退出控制
            if cv2.waitKey(1) & 0xFF == ord('q'): break
               
    except Exception as e:
        print(f" 主循环异常: {str(e)}")
    except KeyboardInterrupt:
        print(" 收到中断信号...")

    #资源清理
    finally:
        print(" 正在释放资源...")
        camera.release()
        try:
            stepper_yaw.close()
            stepper_pitch.close()
        except Exception as e:
            print(f" 电机关闭异常: {e}")
        
        #释放所有GPIO
        GPIO.cleanup()

        cv2.destroyAllWindows()
        print(" 系统已安全关闭")


if __name__ == "__main__":
    main()    