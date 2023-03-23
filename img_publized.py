#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

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

if __name__ == '__main__':
    rospy.init_node('image_receiver', log_level=rospy.INFO)
    image_receiver = ImageReceiver()

    while not rospy.is_shutdown():
        image = image_receiver.get_image()
        if image is not None:
            # Process the image as needed
            pass
