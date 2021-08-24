import cv2
import numpy as np
import rospy
import torch
from models.experimental import attempt_load
from sensor_msgs.msg import Image
from utils.general import non_max_suppression
import matplotlib.pyplot as plt


class CircleTracking:
    def __init__(self):
        # Initialize
        self.device = torch.device('cpu')
        self.half = False and self.device.type != 'cpu'

        # Load self.model
        weights = "weights/best.pt"
        self.model = attempt_load(weights, map_location=self.device)

        # Run inference
        if self.device.type != 'cpu':
            self.model().to(self.device).type_as(next(self.model.parameters()))

        self.size = (864, 512)

        rospy.Subscriber("/camera_front/color/image_raw", Image, self.circle_detector)

    def circle_detector(self, data):
        cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        img = cv2.resize(cv_image, self.size)
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        pred = self.model(img)[0]
        pred = non_max_suppression(pred)
        return pred


if __name__ == '__main__':
    rospy.init_node('yolo')
    CircleTracking()
    rospy.spin()
