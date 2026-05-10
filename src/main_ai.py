import cv2
import time
import threading
import queue
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
    cv2.createTrackbar('onfire_tol', 'Controls', 5, 100, nothing) 
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
    
    pid_yaw.set_Kp(yaw_kp)
    pid_yaw.set_Ki(yaw_ki)
    pid_yaw.set_Kd(yaw_kd)
    
    pid_pitch.set_Kp(pitch_kp)
    pid_pitch.set_Ki(pitch_ki)
    pid_pitch.set_Kd(pitch_kd)

    return vel_rpm, acc, yaw_sentry_speed

# ================= 线程A：跟踪解算 + PID + 电机控制 =================
def thread_a_worker():
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        # 获取最新UI控制参数
        with param_lock:
            vel_rpm = control_params['vel_rpm']
            acc = control_params['acc']
            yaw_sentry_speed = control_params['yaw_sentry_speed']

        # 目标检测
        target = detector.detect(frame)
        
        # 滤波跟踪与解算
        yaw, pitch, dist, status, laser_pos = tracker.track(target)
        
        # 同步 raw 帧（原逻辑保留）
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
        lazer.set_value(1 if is_onfire else 0)
        
        # 保持独立循环频率，避免CPU空转满载
        time.sleep(0.015) 

def main():
    print("视觉跟踪系统启动... 按 'q' 键退出。")
    init_board()

    # 启动独立线程
    t_a = threading.Thread(target=thread_a_worker, name="ThreadA_Tracking", daemon=True)
    t_b = threading.Thread(target=thread_b_worker, name="ThreadB_GPIO", daemon=True)
    t_a.start()
    t_b.start()

    prev_time = time.time()
    try:
        while not stop_event.is_set():
            # 读帧
            ret, frame = camera.read()
            if not ret:
                break

            # 更新UI参数
            vel_rpm, acc, yaw_sentry_speed = update_params()
            
            # 同步至共享字典供线程A读取
            with param_lock:
                control_params['vel_rpm'] = vel_rpm
                control_params['acc'] = acc
                control_params['yaw_sentry_speed'] = yaw_sentry_speed
            
            # 投递帧到处理队列（非阻塞丢弃积压帧，保证实时性）
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except queue.Empty: pass
            try: frame_queue.put_nowait(frame)
            except queue.Full: pass

            # 计算 FPS
            curr_time = time.time()
            fps = 1.0 / max(curr_time - prev_time, 1e-6)
            prev_time = curr_time

            # 读取跟踪结果用于显示与打印
            with result_lock:
                yaw = tracking_result['yaw']
                pitch = tracking_result['pitch']
                dist = tracking_result['dist']
                status = tracking_result['status']
                laser_pos = tracking_result['laser_pos']
            
            # 格式化状态文本
            if status == Status.TRACK:
                info = f"[TRACK] Yaw:{yaw:.2f} Pitch:{pitch:.2f} Dist:{dist:.1f}cm"
            elif status == Status.TMP_LOST:
                info = f"[PREDICT] Predicting... Dist:{dist:.1f}cm"
            else:
                info = "[LOST] Searching..."
            
            print(f"FPS: {fps:.1f} | {info}")
            
            # 绘制与展示 (OpenCV GUI 必须保持在主线程)
            if show_windows:
                vis_det, bin_img = detector.display(dis=1)
                vis_trk = tracker.display(dis=1, laser_pos=laser_pos)
                if vis_det is not None:
                    cv2.imshow("DETECTOR", vis_det)
                if bin_img is not None:
                    cv2.imshow("BIN", bin_img)
                if vis_trk is not None:
                    cv2.imshow("Tracker", vis_trk)
            else:
                cv2.destroyAllWindows()
            
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break
            
    except KeyboardInterrupt:
        print(" 收到中断信号...")
    except Exception as e:
        print(f" 主循环异常: {str(e)}")
    finally:
        print(" 正在释放资源...")
        stop_event.set()  # 通知子线程安全退出
        t_a.join(timeout=2.0)
        t_b.join(timeout=2.0)

        camera.release()
        try:
            stepper_yaw.close()
            stepper_pitch.close()
        except Exception as e:
            print(f" 电机关闭异常: {e}")
        cv2.destroyAllWindows()
        lazer.cleanup()
        heart_beat.cleanup()
        print(" 系统已安全关闭")

if __name__ == '__main__':
    main()
