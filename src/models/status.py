import Hobot.GPIO as GPIO
import time

# 全局初始化（官方示例标准写法，避免多实例重复配置冲突）
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

class GPIN:
    def __init__(self, pin=1, mode=1):
        self.pin = pin
        self.mode = mode
        self.status = GPIO.LOW  # 软件电平缓存，减少无效总线写入
        # 主线程心跳检测
        self.last_heartbeat = time.time()
        self.timeout = 3.0  # 3秒超时熔断
        
        # 呼吸灯状态机（改为时间驱动，固定3秒周期，移除原step/_counter）
        self._breath_start = 0  # 呼吸周期起始时间戳
        self._pwm_cnt = 0       # PWM输出细分计数器
        self.duty = 0           # 当前目标占空比 (0~100)

        # 严格模式隔离配置
        if self.mode == 1:
            GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        elif self.mode == 0:
            GPIO.setup(self.pin, GPIO.IN)

    def set_value(self, value):
        # 输出模式专属，拦截误写
        if self.mode != 1:
            print(f"错误: 引脚 {self.pin} 是输入模式，无法设置值")
            return
        # 开火极简映射
        self.status = GPIO.LOW if value == 1 else GPIO.HIGH
        GPIO.output(self.pin, self.status)

    def read_status(self):
        # 输入模式专属，拦截误读
        if self.mode == 1:
            print(f"错误: 引脚 {self.pin} 是输出模式，无法读取状态")
            return None
        # 返回1或0, 1 表示按下(低电平)，0 表示松开(高电平)
        return int(GPIO.input(self.pin) == GPIO.LOW)

    def heartbeat(self):
        # 刷新主线程存活时间戳
        self.last_heartbeat = time.time()

    def flash(self):
        # 外部主循环每调用一次，内部推进一帧。绝不 sleep 阻塞业务
        self.heartbeat()
        self._update_breathing()

    def _update_breathing(self):
        # 超时熔断：主线程卡死>3秒，强制拉低灭灯
        if time.time() - self.last_heartbeat > self.timeout:
            GPIO.output(self.pin, GPIO.LOW)
            self._breath_start = 0  # 重置起点，等待下次恢复
            return

        # 记录呼吸周期起点（首次调用或熔断后重置）
        if self._breath_start == 0:
            self._breath_start = time.time()

        # 固定3秒一个完整周期 (0->100->0)，相位计算完全不受主循环FPS影响
        elapsed = time.time() - self._breath_start
        phase = (elapsed % 3.0) / 3.0  # 归一化到 0.0 ~ 1.0
        
        # 三角波计算：前1.5秒占空比线性上升，后1.5秒线性下降
        self.duty = int(phase * 200) if phase < 0.5 else int((1.0 - phase) * 200)

        # 软件PWM输出：每调用一次刷新一次电平，主循环越快视觉越平滑
        self._pwm_cnt = (self._pwm_cnt + 1) % 100
        GPIO.output(self.pin, GPIO.HIGH if self._pwm_cnt < self.duty else GPIO.LOW)

    def cleanup(self):
        # 安全释放，避免残留高电平
        try: GPIO.output(self.pin, GPIO.LOW)
        except RuntimeError: pass
        GPIO.cleanup()