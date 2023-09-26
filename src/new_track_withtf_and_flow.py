#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
from ultralytics import YOLO
import rospy
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PointStamped
from hera_tracker.msg import riskanddirection
from std_msgs.msg import Int32
import traceback


class Tracker:
    def __init__(self) -> None:
        self.model = YOLO('yolov8n.pt')
        image_topic = "/camera/rgb/image_raw"
        self.id_to_follow = 1
        self.last_centroid_x = None
        self._global_frame = "map"
        self._current_image = None
        self.time = rospy.Time.now()
        point_cloud_topic = "/camera/depth/points"
        self.publish_tf = None
        self._tf_listener = tf.TransformListener()
        self._current_pc = None
        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self._risk_and_direction_pub = rospy.Publisher('~risk_and_direction', riskanddirection, queue_size=10)
        self._person_to_follow_pub = rospy.Publisher('person_to_follow', PointStamped, queue_size=10)
        self.risk_matrix = [
            [9, 8, 7], [7, 6, 5], [5, 4, 3], [2, 1, 2], [3, 4, 5], [5, 6, 7], [7, 8, 9]
        ]
        self._imagepub = rospy.Publisher('~objects_label', Image, queue_size=10)
        if point_cloud_topic is not None:
            rospy.Subscriber(point_cloud_topic, PointCloud2, self.pc_callback)
        else:
            rospy.loginfo('No point cloud information available. Objects will not be placed in the scene.')
        self._tfpub = tf.TransformBroadcaster()
        rospy.loginfo('Ready to Track!')

    def read_published_tf(self, tf_id, frame):
        try:
            (trans, rot) = self._tf_listener.lookupTransform(frame, tf_id, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

    def image_callback(self, image):
        self._current_image = image

    def pc_callback(self, pc):
        self._current_pc = pc

    def calculate_tf(self, current_centroid_x, current_centroid_y):
        point_z, point_x, point_y = None, None, None
        (trans, _) = self._tf_listener.lookupTransform('/' + self._global_frame, '/xtion', rospy.Time(0))
        if self._current_pc is None:
            rospy.loginfo('No point cloud')
        pc_list = list(
            pc2.read_points(self._current_pc,
                            skip_nans=True,
                            field_names=('x', 'y', 'z'),
                            uvs=[(current_centroid_x, current_centroid_y)]))
        if len(pc_list) > 0:
            self.publish_tf = True
            tf_id = 'person_follow'
            point_z, point_x, point_y = pc_list[0]
        if point_z is not None and point_x is not None and point_y is not None:
            object_tf = [point_y, -point_z, point_x]
            frame = 'xtion'
            if self._global_frame is not None:
                object_tf = np.array(trans) + object_tf
                frame = self._global_frame
            if object_tf is not None:
                self._tfpub.sendTransform(object_tf,
                                          tf.transformations.quaternion_from_euler(0, 0, 0),
                                          self.time,
                                          tf_id,
                                          frame)
                self.time = self.time + rospy.Duration(1e-3)
            trans, _ = self.read_published_tf(tf_id, frame)
            if trans:
                point_msg = PointStamped()
                point_msg.header.frame_id = frame
                point_msg.header.stamp = self.time
                point_msg.point.x = trans[0]
                point_msg.point.y = trans[1]
                point_msg.point.z = 0.0
                self._person_to_follow_pub.publish(point_msg)

    def calculate_centroid(self, bbox):
        x_min, y_min, x_max, y_max = bbox
        return int((x_min + x_max) / 2), int((y_min + y_max) / 2)
    def calculate_risk(self, centroid, frame_width):
        segment_width = frame_width // 7
        segment = centroid[0] // segment_width
        direction = "STATIONARY"
        if self.last_centroid_x:
            if centroid[0] < self.last_centroid_x:
                direction = "LEFT"
            elif centroid[0] > self.last_centroid_x:
                direction = "RIGHT"
        self.last_centroid_x = centroid[0]
        direction_map = {"LEFT": 0, "STATIONARY": 1, "RIGHT": 2}
        risk_value = self.risk_matrix[segment][direction_map[direction]]
        # Publish risk and direction
        msg = riskanddirection()
        msg.direction = direction
        msg.risk.data = risk_value
        self._risk_and_direction_pub.publish(msg)
        return risk_value
    def main_track(self):
        # cap = cv2.VideoCapture(self.source)
        frame_width = 640
        rate = rospy.Rate(30)  # 30 Hz or 30 fps
        while not rospy.is_shutdown():
            if self._current_image is not None:
                
                small_frame = self._bridge.imgmsg_to_cv2(self._current_image, desired_encoding='bgr8')
                
                results = self.model.track(source = small_frame, persist=True, classes=0, verbose=False, device=0)
                annotated_frame = results[0].plot()
                for r in results:
                    if hasattr(r, 'boxes') and r.boxes and hasattr(r.boxes, 'id') and r.boxes.id is not None:
                        ids = r.boxes.id.tolist()
                        bbox_coords_list = r.boxes.xyxy.tolist()
                        for detected_id, bbox_coords in zip(ids, bbox_coords_list):
                            if detected_id == self.id_to_follow:
                                centroid = self.calculate_centroid(bbox_coords)
                                cx, cy = centroid
                                risk_value = self.calculate_risk(centroid, frame_width)
                                print(f"Risk value for ID {self.id_to_follow}:", risk_value)
                                self.calculate_tf(cx, cy)
                                print(f"Centroid for ID {self.id_to_follow}:", centroid)
                                cv2.circle(annotated_frame, centroid, 5, (0, 255, 0), -1)
                cv2.imshow("YOLOv8 Tracking", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                rate.sleep()
            else:
                print("No image received")
                rate.sleep()
        cv2.destroyAllWindows()


if __name__ == "__main__":    
    rospy.init_node('tracker_node', anonymous=True)
    try:
        Tracker().main_track()
    except KeyboardInterrupt:
        print("\nEnd of the program :)")
    except rospy.ROSInterruptException:
        pass
