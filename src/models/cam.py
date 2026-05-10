import cv2

class Camera:
    def __init__(self, index = 4, width=640, height=480):
        index = self.find_index(index)
        if index is None:
            raise RuntimeError("不能打开任何摄像头")
        self.cam = cv2.VideoCapture(index, cv2.CAP_V4L2)

        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)     
        # 获取实际生效的画面宽度与高度
        self.width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
  
    def read(self):
        ret, frame = self.cam.read()
        return ret, frame
    
    def find_index(self, index=4):
        max_index = index + 20
        for i in range(index, max_index):
            cam = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cam.isOpened():
                cam.release()
                return i
            cam.release()
        return None
    
    def release(self):
        self.cam.release()