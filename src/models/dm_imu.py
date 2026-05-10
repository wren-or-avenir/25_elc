from .dm_serial import DM_Serial
class IMU:
    def __init__(self, port, baud = 921600):
        self.dev = DM_Serial(port, baud)
        self.dev.start_reader(read_sleep=0.001)
        self.offset_yaw = 0.0      # yaw轴校准偏移
        self.offset_pitch = 0.0    # pitch轴校准偏移

    def solve_abs(self, vis_yaw, vis_pitch):
        pkt, _, _ = self.dev.get_latest()

        # 降级,无IMU时退回纯视觉
        if pkt is None:
            return vis_yaw, vis_pitch  
        
        _, (r, p, y) = pkt  # 单位(度)

        # 绝对角度 = IMU读数 + 视觉读数 + 校准偏移
        abs_yaw = y + vis_yaw + self.offset_yaw
        abs_pitch = p + vis_pitch + self.offset_pitch

        # 将绝对角度归一化
        abs_yaw = (abs_yaw + 180) % 360 - 180
        abs_pitch = max(-90.0, min(90.0, abs_pitch))

        return abs_yaw, abs_pitch

    def get_abs(self, vis_yaw, vis_pitch):
        """ 对外接口,返回绝对云台角度(yaw, pitch) """
        abs_yaw, abs_pitch = self.solve_abs(vis_yaw, vis_pitch)
        return abs_yaw, abs_pitch
    
imu = IMU(port='/dev/ttyACM0')  # 替换为实际的串口号