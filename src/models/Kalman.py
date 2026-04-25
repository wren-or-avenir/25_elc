import cv2
import numpy as np

class KalmanFilter:
    def __init__(self, q_scale=0.01, r_scale=0.5):
        """
        q_scale: 过程噪声缩放因子（信任模型的程度）
        r_scale: 测量噪声缩放因子（信任传感器的程度）
        卡尔曼增益k:  约等于p/(p+r)
        q/r越大,k越大,滤波器越依赖测量值(更灵敏);
        q/r越小,k越小,滤波器越依赖预测值(更平稳)
        """
        self.q_scale = q_scale 
        self.r_scale = r_scale
        self.dt = 0.033     
        
        # 状态维度2(位置,速度)，观测维度1(位置)
        self.kf = cv2.KalmanFilter(2, 1)
        
        # 状态转移矩阵 (F)
        self.kf.transitionMatrix = np.array([[1, self.dt],
                                             [0, 1]], np.float32)
        
        # 观测矩阵 (H)
        self.kf.measurementMatrix = np.array([[1, 0]], np.float32)
        
        # 过程噪声协方差 (Q)
        self.kf.processNoiseCov = np.eye(2, dtype=np.float32) * q_scale
        # 如果云台跟不上目标（响应滞后），尝试增大此值（如 0.1）。
        # 如果预测轨迹太激进，尝试减小此值（如 0.001）。
        
        # 测量噪声协方差 (R)
        self.kf.measurementNoiseCov = np.array([[r_scale]], np.float32)
        # 如果激光点跟随目标时剧烈抖动，增大此值（如 1.0 或 5.0）来平滑数据。
        # 如果激光点非常平滑但反应太慢，减小此值（如 0.1）。
        
        # 初始状态及不确定性
        # 初始化协方差为较大值，确保第一帧能迅速锁死目标
        self.reset()

    def predict(self, dt=None):
        if dt is not None:
            self.dt = dt
            # 动态更新状态转移矩阵中的 dt 部分
            self.kf.transitionMatrix[0, 1] = dt
        prediction = self.kf.predict()
        return prediction[0, 0] , prediction[1, 0] # 返回预测的位置值

    def update(self, measurement):
        measure = np.array([[np.float32(measurement)]])
        estimate = self.kf.correct(measure)
        return estimate[0, 0], estimate[1, 0] # 返回修正后的最优位置值

    def reset(self):
        # 状态置零，初始化
        self.kf.statePost = np.zeros((2, 1), np.float32)
        # 协方差重置为大值：告诉滤波器“我现在完全不确定位置”，强制它快速收敛到第一帧观测值
        self.kf.errorCovPost = np.eye(2, dtype=np.float32) * 1000.0

    def get_state(self):
        return self.kf.statePost[0, 0], self.kf.statePost[1, 0]