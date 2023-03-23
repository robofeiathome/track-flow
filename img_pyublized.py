import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import processing as pr
import os
import sys
import platform
import numpy as np
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn


class ImageReceiver:
    def __init__(self):
        image_topic = '~/zed2/zed_node/left_raw/image_raw_color'

        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self._current_image = None

        rospy.loginfo('Ready to receive images!')

    def image_callback(self, image):
        """Image callback"""
        self._current_image = image

    def get_image(self):
        if self._current_image is not None:
            try:
                return self._bridge.imgmsg_to_cv2(self._current_image, desired_encoding='bgr8')
            except CvBridgeError as e:
                print(e)
        return None

    def preprocess(self, img):
        # convert the image to float type
        img = img.astype(np.float32)
        # scale the pixel values from [0, 255] to [0.0, 1.0]
        img /= 255.0
        return img

    def vide_show(self, img):
        # define the window name as a string
        window_name = "Image"

        # create a list of windows if it doesn't exist
        if not hasattr(self, "windows"):
            self.windows = []

        # check the operating system
        if platform.system() == "Linux":
            # resize the window (Linux only)
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow(window_name, img.shape[1], img.shape[0])

        # show the image in the window
        cv2.imshow(window_name, img)

        # exit if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            sys.exit()


if __name__ == '__main__':
    rospy.init_node('image_receiver', log_level=rospy.INFO)
    image_receiver = ImageReceiver()

    while not rospy.is_shutdown():
        image = image_receiver.get_image()
        if image is not None:
            img = image_receiver.preprocess(image)
            return img
            #image_receiver.vide_show(img)
            # Process the image as needed
