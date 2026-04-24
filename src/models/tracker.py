import math
import numpy as np
from .Kalman import KalmanFilter
import time
from enum import IntEnum
from .dm_imu import imu 
import cv2

# 追踪状态展示
class Status(IntEnum):
    LOST = 0        # 完全丢失目标
    TMP_LOST = 2    # 临时丢失目标，仍在预测中
    TRACK = 3       # 正常跟踪目标

class Tracker:
    def __init__(self, img_width=640, img_height=480, vfov=48.0, hfov =80.0,  real_height=17.5,  f_pixel_h=725.6, use_kf = False):
        self.img_width = img_width
        self.img_height = img_height
        self.vfov = vfov # 相机垂直视场角 
        self.hfov = hfov # 相机水平视场角 
        self.f_pixel_h = f_pixel_h # 标定得到的垂直焦距 (像素)
        self.real_height = real_height # 目标物理高度 (cm)
        
        self.use_kf = use_kf    # 是否启用卡尔曼滤波器
        self.kf_cx = KalmanFilter(q_scale=0.35, r_scale=0.1)    # cx 卡尔曼滤波器
        self.kf_cy = KalmanFilter(q_scale=0.35, r_scale=0.1)    # cy 卡尔曼滤波器
        self.kf_dist = KalmanFilter(q_scale=0.01, r_scale=2.0)  # distance 卡尔曼滤波器

        self.lost_count = 0  # 丢帧数
        self.frame_lost_tol = 8   # 丢帧容忍度 
        self.last_time = None

        self.onfire = False # 开火状态，默认为关闭
        self.onfire_tol = 3 # 开火容忍，单位为度

        self.raw = None
        # 物理偏移补偿 (单位: cm)
        self.ref_point = np.array([0.0, 1.4, 0.0])      # 测量得出激光笔位于相机光轴上方1.4cm,视差补偿为 np.array([0.0, 1.4, 0.0])
        self.status = Status.LOST # 初始状态为 LOST

    def time_diff(self):
        # 起到一个动态dt的作用
        current_time = time.time_ns()
        # 首次调用时没有上次时间，返回一个默认的dt
        if self.last_time is None:
            self.last_time = current_time
            return 0.033
        else :
            diff = (current_time - self.last_time) 
            self.last_time= current_time
            return diff / 1e9  # 转换为秒
        
    def get_dist(self, board):
        # 计算距离
        pts = board.points
        h_left = math.sqrt((pts[0][0]-pts[1][0])**2 + (pts[0][1]-pts[1][1])**2)
        h_right = math.sqrt((pts[3][0]-pts[2][0])**2 + (pts[3][1]-pts[2][1])**2)
        # h_left = math.sqrt((pts[0][0]-pts[3][0])**2 + (pts[0][1]-pts[3][1])**2)
        # h_right = math.sqrt((pts[1][0]-pts[2][0])**2 + (pts[1][1]-pts[2][1])**2)
        avg_h_px = (h_left + h_right) / 2.0
        # 除零保护
        if avg_h_px < 1: 
            return 1000.0
        dist = (self.real_height * self.f_pixel_h) / avg_h_px
        return dist      

    def filter(self, target):
        # 暂存变量
        cx, cy, dist = 0, 0, 0.0
        # 接口变量
        filtered_center = (0, 0)
        filtered_dist = 0.0

        #动态计算帧率，避免固定dt导致的预测不准确问题
        dt = self.time_diff()

        if self.use_kf:
            if target is not None and target.center is not None:
                # 丢帧计数重置
                self.lost_count = 0
                if self.status == Status.LOST:
                    self.status = Status.TRACK  # 状态调整
                    self.kf_cx.reset()
                    self.kf_cy.reset()
                    self.kf_dist.reset()
                else:
                    self.status = Status.TRACK  # 状态调整
                # 预测
                pred_cx, _ = self.kf_cx.predict(dt)
                pred_cy, _ = self.kf_cy.predict(dt)
                pred_dist, _ = self.kf_dist.predict(dt)
                # 更新
                update_cx, _ = self.kf_cx.update(target.center[0])
                update_cy, _ = self.kf_cy.update(target.center[1])
                update_dist, _ = self.kf_dist.update(self.get_dist(target))
                cx, cy, dist = update_cx, update_cy, update_dist
            else:
                #丢帧，开始计数
                self.lost_count += 1
                if self.lost_count <= self.frame_lost_tol:
                    self.status = Status.TMP_LOST   # 状态调整
                     # 预测
                    pred_cx, _ = self.kf_cx.predict(dt)
                    pred_cy, _ = self.kf_cy.predict(dt)
                    pred_dist, _ = self.kf_dist.predict(dt)
                    cx, cy, dist = pred_cx, pred_cy, pred_dist
                else:
                    self.status = Status.LOST   # 状态调整
                    # 丢帧过多，重置滤波器状态
                    self.kf_cx.reset()
                    self.kf_cy.reset()
                    self.kf_dist.reset()
                    # 防止云台乱甩
                    cx, cy, dist = self.img_width / 2, self.img_height / 2, 0.0
        else:
            if target is not None:
                cx, cy = target.center
                dist = self.get_dist(target)
                self.status = Status.TRACK
            else:
                cx, cy = self.img_width / 2, self.img_height / 2
                dist = 0.0
                self.status = Status.LOST

        filtered_center = (cx, cy)
        filtered_dist = dist
        
        return filtered_center, filtered_dist

    def solve(self, center, dist):
        dist = dist if dist > 0 else 0.1
        u, v = center
        # 计算像素坐标相对于图像中心的偏移量
        offset_x = u - self.img_width / 2
        offset_y = v - self.img_height / 2

        # 考虑视差补偿,目前视差补偿为0，后续如果装激光笔了再调整
        dx = offset_x - (self.ref_point[0] * self.f_pixel_h / dist if dist != 0 else 0)
        dy = offset_y - (self.ref_point[1] * self.f_pixel_h / dist if dist != 0 else 0)

        # 激光坐标
        laser_u = self.img_width / 2 + dx
        laser_v = self.img_height / 2 + dy
        # 计算云台角度
        # Yaw: 逆时针为正 
        yaw = -math.degrees(math.atan2(dx, self.f_pixel_h))
        # Pitch: 向下为正
        pitch = math.degrees(math.atan2(dy, self.f_pixel_h))

        return yaw, pitch, dist, (int(laser_u), int(laser_v))
    
    def check_onfire(self, pitch, yaw):
        arrived = False
        if abs(pitch) < self.onfire_tol and abs(yaw) < self.onfire_tol:
            arrived = True
        else:
            arrived = False
        return arrived
       
    def track(self, board):
        """对外接口，追踪主逻辑"""
        if board is not None and board.center is not None:
            filtered_center, filtered_dist = self.filter(board)
            if self.status != Status.LOST:
                rel_yaw, rel_pitch, dist, laser_pos = self.solve(filtered_center, filtered_dist)
                self.onfire = self.check_onfire(rel_pitch, rel_yaw)
                yaw, pitch = imu.get_abs(rel_yaw, rel_pitch)
                self.laser_pos = laser_pos
                return yaw, pitch, dist, self.status, laser_pos
            else:
                self.laser_pos = None
                return 0.0, 0.0, 0.0, self.status, None
        else:
            self.filter(None)
            self.laser_pos = None
            return 0.0, 0.0, 0.0, self.status, None
        
    def display(self, dis=1, laser_pos=None):
        """对外接口，展示滤波后的结果"""
        if self.raw is None:
            return None
        vis = self.raw.copy()
        if dis == 1 and laser_pos is not None:
            
            h, w = vis.shape[:2]
            cv2.circle(vis, (w // 2, h // 2), 5, (0, 165, 255), -1)

            cx, cy = laser_pos
            color = (255, 0, 255)
            cv2.line(vis, (cx - 10, cy), (cx + 10, cy), color, 2)
            cv2.line(vis, (cx, cy - 10), (cx, cy + 10), color, 2)
            cv2.circle(vis, (cx, cy), 3, color, -1)
        return vis