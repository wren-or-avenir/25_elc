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

#硬件全局实例化
'''
要实例的有
cam ,detector, tracker, stepper_yaw, stepper_pitch, 需要的GPIO
顺序逻辑：数据源(摄像头) → 处理源(识别) → 决策源(跟踪) → 执行器(电机控制)
'''
cam = Camera.Camera(index = 4, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5, use_kf = True)
stepper_yaw = Stepper.EmmMotor(port ='COM20', baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.EmmMotor(port ='COM7', baudrate = 115200, timeout = 1, motor_id = 2)
#heart_beat = GPIN.GPIN(pin=13, mode=1) #呼吸灯，用于表示主程序还在跑
pid_yaw = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)
pid_pitch = pid.PIDController(Kp = 5, Ki = 5, Kd = 5, dt = 1/30)

def nothing(x):
    pass
#UI
def init_board():
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Result', 540, 180)
    cv2.namedWindow('Controls', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Controls', 0, 0)
    cv2.resizeWindow('Controls', 320, 500)
    '''
    创建滑动条
    '''
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)

    cv2.createTrackbar('yaw_kp', 'Controls', 2, 100, nothing)   # 范围 0.00 - 1.00
    cv2.createTrackbar('yaw_ki', 'Controls', 0, 100, nothing)   # 范围 0.000 - 0.100 
    cv2.createTrackbar('yaw_kd', 'Controls', 1, 100, nothing)  # 范围 0.000 - 0.010

    cv2.createTrackbar('pitch_kp', 'Controls', 2, 100, nothing)   # 范围 0.00 - 1.00
    cv2.createTrackbar('pitch_ki', 'Controls', 0, 100, nothing)   # 范围 0.000 - 0.100 
    cv2.createTrackbar('pitch_kd', 'Controls', 1, 100, nothing)  # 范围 0.000 - 0.010

    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)
    cv2.createTrackbar('acc', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)
#读原始值——数据转换——赋值给模块——返回控制参
def update_hsv():
    #读原始值
    current_thresh = cv2.getTrackbarPos('Threshold', 'Controls')

    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls')/100
    yaw_ki = cv2.getTrackbarPos('yaw_ki', 'Controls')/10000
    yaw_kd = cv2.getTrackbarPos('yaw_kd', 'Controls')/10000

    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls')/100
    pitch_ki = cv2.getTrackbarPos('pitch_ki', 'Controls')/10000
    pitch_kd = cv2.getTrackbarPos('pitch_kd', 'Controls')/10000

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

def main ():
    init_board()
    if not cam.cam.isOpened():
        print("摄像头打不开")
        return
    ast_frame_time = time.time()
    print(" 系统启动，按 'q' 退出...")

    try:
        while True:
            #呼吸灯，证明主程序在运行(单线程中闪烁频率完全受制于主循环的运行速度)
            #heart_beat.flash()
            '''
            读帧-更新参数-识别目标-计算角度-控制电机-显示画面-延时与退出
            '''
            #读帧
            ret, frame = cam.cam.read()
            if not ret: 
                print("无法读帧")
                break

            # 计算 FPS（终端打印）
            curr_time = time.time()
            fps = 1.0 / max(curr_time - prev_time, 1e-6)
            prev_time = curr_time

            #更新参数
            vel_rpm, acc = update_hsv()

            #识别目标
            target = detector.detect(frame)

            '''
            控制电机
            '''
            res = tracker.track(target)
            yaw, pitch, dist, status, laser_pos = res

            # 格式化状态文本（终端打印）
            if status == Status.TRACK:
                info = f"[TRACK] Yaw:{yaw:.2f} Pitch:{pitch:.2f} Dist:{dist:.1f}cm"
            elif status == Status.TMP_LOST:
                info = f"[PREDICT] Predicting... Dist:{dist:.1f}cm"
            else:
                info = "[LOST] Searching..."
                    
            # 运行电机
            if status in (Tracker.Status.TRACK, Tracker.Status.TMP_LOST):#能识别+预测
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

            elif status == Tracker.Status.LOST:#丢帧超多阈值，停止运动
                #重置pid
                pid_yaw.reset()
                pid_pitch.reset()
                pass

                    
            # 显示与退出
            cv2.imshow('bin', detector.last_binary)
            cv2.imshow('Result', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    except Exception as e:
        print(f" 主循环异常: {str(e)}")
    except KeyboardInterrupt:
        print(" 收到中断信号...")
    
    #资源清理
    finally:
        print(" 正在释放资源...")
        cam.cam.release()
        try:
            stepper_yaw.close()
            stepper_pitch.close()
        except Exception as e:
            print(f" 电机关闭异常: {e}")
        cv2.destroyAllWindows()
        print(" 系统已安全关闭")


if __name__ == "__main__":
    main()