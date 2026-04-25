#pid模块
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=1/30):
        self.Kp = Kp  # 比例增益
        self.Ki = Ki  # 积分增益
        self.Kd = Kd  # 微分增益
        self.dt = dt  # 时间步长

        self.last_error = 0.0#上一次的误差,用于计算D
        self.integral = 0.0#积分累积值,用于计算I
        self.last_time = time.time()#上一次计算的时间戳，因为I和D的计算都要用到时间差
        
        # 抗积分饱和（防止I项过大导致失控）
        self.integral_limit = 100.0

    def compute(self, error):
        '''
        输入: error (当前误差，例如 yaw 角度偏差)
        返回: output (控制量，例如电机需要转动的角度增量)
        '''
        current_time = time.time()#这一次的时间
        dt = current_time - self.last_time

        # 防止 dt 过小或为负数 
        if dt <= 0 or dt > 1.0: 
            dt = 1/30.0 

        # --- P (比例) ---
        p_out = self.Kp * error

        # --- I (积分) ---
        self.integral += error * dt
        # 限制积分项的大小，防止失控
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < - self.integral_limit:
            self.integral = - self.integral_limit
        i_out = self.Ki * self.integral

        # --- D (微分) ---
        # 避免除以零
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0
        d_out = self.Kd * derivative

        # --- 更新状态 ---
        self.last_error = error
        self.last_time = current_time

        '''
        总输出
        '''
        output = p_out + i_out + d_out
        return output
    
    def reset(self):
        """重置状态"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def set_Kp(self, Kp):
        """动态设置比例增益"""
        self.Kp = Kp

    def set_Ki(self, Ki):
        """动态设置积分增益"""
        self.Ki = Ki

    def set_Kd(self, Kd):
        """动态设置微分增益"""
        self.Kd = Kd