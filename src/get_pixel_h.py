import cv2
import numpy as np
import math
from models.detector import Detector 

def calibrate_focal_length():
    # --- 1. 手动设置已知物理参数 ---
    # 确保你标定时，卷尺量的距离和这里一致
    REAL_DIST = 90.0   # 距离 (cm)
    REAL_HEIGHT = 17.5  # 靶纸黑框的真实物理高度 (cm)
    
    cap = cv2.VideoCapture(4)
    # 强制设置分辨率，必须与未来比赛运行的分辨率严格一致
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # --- 2. 初始化检测器 ---
    # 根据你提供的 Detector 定义：只接受 min_area 和 max_area
    detector = Detector(min_area=5000, max_area=500000)

    print("--- 标定程序已启动 ---")
    print(f"目标距离设定: {REAL_DIST}cm, 目标物理高度: {REAL_HEIGHT}cm")
    print("操作说明: 按 's' 键确认并保存焦距, 按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        # 1. 运行你的检测逻辑
        # 你的函数返回：渲染后的图 和 board 对象
        annotated_frame, board = detector.process_image(frame)

        height_px = 0
        if board.is_valid and len(board.points) == 4:
            # --- 3. 计算像素高度 ---
            # 你的 points 顺序为: [左上, 左下, 右下, 右上]
            # 计算左侧边和右侧边的平均长度作为像素高度
            pts = board.points
            h_left = math.sqrt((pts[0][0] - pts[1][0])**2 + (pts[0][1] - pts[1][1])**2)
            h_right = math.sqrt((pts[3][0] - pts[2][0])**2 + (pts[3][1] - pts[2][1])**2)
            height_px = (h_left + h_right) / 2.0

            # 2. 实时计算理论焦距 (用于预览)
            # 公式: f = (D * H_px) / H_real
            current_f = (REAL_DIST * height_px) / REAL_HEIGHT
            
            # 在画面上显示结果
            cv2.putText(annotated_frame, f"Current F: {current_f:.2f}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Height_px: {height_px:.1f}", (20, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 显示检测画面和二值化画面（方便调阈值）
        cv2.imshow("Calibration (Annotated)", annotated_frame)
        if detector.last_binary is not None:
            cv2.imshow("Binary Mask (Check this)", detector.last_binary)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and height_px > 0:
            final_f = (REAL_DIST * height_px) / REAL_HEIGHT
            print(f"\n[标定成功!]")
            print(f"检测到像素高度: {height_px:.2f} px")
            print(f"您的像素焦距 f_pixel_h 为: {final_f:.2f}")
            print("-" * 30)
            print(f"请在 Tracker 类中使用这个数字: self.f_pixel_h = {final_f:.2f}")
            break
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate_focal_length()