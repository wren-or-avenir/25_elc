import cv2
import time
from models.detector import Detector
from models.tracker import Tracker, Status
from models.dm_imu import IMU

# 初始化模块
detector = Detector(min_area=5000, max_area=500000)
tracker = Tracker(f_pixel_h=725.6, real_height=17.5, use_kf=True) # 默认开启卡尔曼
camera_index = 4   # 相机索引

# 用于记录上一次的阈值，实现去重打印
last_thresh = -1

def nothing(x):
    pass

def init_board():
    """初始化调试窗口和滑动条"""
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 300, 150)
    cv2.namedWindow('DETECTOR', cv2.WINDOW_FREERATIO)  
    cv2.namedWindow('BIN', cv2.WINDOW_FREERATIO)      
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)

def update_params():
    """回调获取滑块参数，若数值改变则在终端打印"""
    global last_thresh
    
    # 获取当前滑动条数值
    current_thresh = cv2.getTrackbarPos('Threshold', 'Controls')
    
    # 同步给检测器
    detector.threshold_value = current_thresh
    
    # 如果数值发生变化，打印到终端
    if current_thresh != last_thresh:
        print(f"[Update] New Threshold: {current_thresh}")
        last_thresh = current_thresh

def main():
    print("视觉跟踪系统启动... 按 'q' 键退出。")
    init_board()

    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 {camera_index}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    prev_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        update_params()
        
        # 目标检测
        target = detector.detect(frame)
        
        # 滤波跟踪与解算
        yaw, pitch, dist, status, laser_pos = tracker.track(target)
        
        # 计算 FPS（终端打印）
        curr_time = time.time()
        fps = 1.0 / max(curr_time - prev_time, 1e-6)
        prev_time = curr_time
        
        # 格式化状态文本（终端打印）
        if status == Status.TRACK:
            info = f"[TRACK] Yaw:{yaw:.2f} Pitch:{pitch:.2f} Dist:{dist:.1f}cm"
        elif status == Status.TMP_LOST:
            info = f"[PREDICT] Predicting... Dist:{dist:.1f}cm"
        else:
            info = "[LOST] Searching..."
        
        # 终端打印 FPS + 状态（每帧输出）
        print(f"FPS: {fps:.1f} | {info}")
        
        # 同步 raw 帧（供 display 使用）
        detector.raw = frame
        tracker.raw = frame
        
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

        # 退出控制
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n" + "="*30)
            print(f"程序退出。最终阈值 Threshold: {detector.threshold_value}")
            print("="*30)
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()