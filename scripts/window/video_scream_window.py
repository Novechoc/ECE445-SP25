# window that includes a video stream and a marked video stream, and a VLM description

import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

class VideoSegmentationApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("实时语义分割")
        self.label = QLabel(self)
        self.label.setFixedSize(640, 480)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # 打开摄像头（也可以替换为视频文件路径）
        self.cap = cv2.VideoCapture(0)

        # 定时器，每 30ms 调用一次 update_frame
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 模拟语义分割（替换为你自己的分割模型）
        segmentation_mask = self.fake_segmentation(frame)

        # 将结果合成：原图加半透明分割结果
        blended = cv2.addWeighted(frame, 0.6, segmentation_mask, 0.4, 0)
        # blended = frame

        # 转换为 QImage 再显示
        rgb_image = cv2.cvtColor(blended, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.label.setPixmap(QPixmap.fromImage(qt_image))

    def fake_segmentation(self, frame):
        """模拟一个伪语义分割蒙版"""
        mask = np.zeros_like(frame)
        h, w, _ = frame.shape
        mask[h//4:3*h//4, w//4:3*w//4] = (0, 255, 0)  # 绿色区域模拟分割结果
        return mask

    def closeEvent(self, event):
        self.cap.release()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoSegmentationApp()
    window.show()
    sys.exit(app.exec_())
