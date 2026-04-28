import Hobot.GPIO as GPIO
import time

_GPIO_MODE_SET = False

class GPIN :

    def __init__(self, pin = 1, mode = 1):
        # 保存参数
        self.pin = pin
        self.mode = mode

        # 初始化
        self.times = 0
        self.status = GPIO.LOW # 当前电平

        # 配置硬件
        if not _GPIO_MODE_SET:#防止多次运行setmode而报错
            GPIO.setmode(GPIO.BOARD)# 设置引脚编号模式为物理引脚号
            GPIO.setwarnings(False)#关闭 GPIO 库的警告信息
            _GPIO_MODE_SET = True

        #模式判断
        if self.mode == 1:
            GPIO.setup(self.pin, GPIO.OUT)#规定输出还是输入：GPIO.OUT(输出),GPIO.IN(输入)
            GPIO.output(self.pin, GPIO.LOW)#仅对输出模式有效:GPIO.HIGH (高电平),GPIO.LOW (低电平)

        elif self.mode == 0:
            GPIO.setup(self.pin, GPIO.IN)
    #输出控制
    def set_value(self, value):
        if self.mode == 1:
            GPIO.output(self.pin, GPIO.HIGH if value == 1 else GPIO.LOW)
        else:
            print(f"错误: 引脚 {self.pin} 是输入模式，无法设置值")
    #闪烁
    def flash(self):
        self.times += 1
        self.times = self.times % 10  # 让计数在 0-9 之间循环

        # 每当 self.times == 0 时电平改变一次
        if self.times == 0:
            if self.status == GPIO.LOW:
                self.status = GPIO.HIGH
            elif self.status == GPIO.HIGH:
                self.status = GPIO.LOW

        GPIO.output(self.pin, self.status)
        

    def read_status(self):
         # 检查模式是否为1，如果是则返回错误
        if self.mode == 1:
            print("错误的引脚类型!!!!!!!!!!!!!!!!!!!!!!!")
            return None
        
        # 检测按钮按下（低电平）
        return int(GPIO.input(self.pin) == GPIO.LOW)