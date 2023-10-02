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
from geometry_msgs.msg import PointStamped
from hera_tracker.msg import riskanddirection
from hera_tracker.srv import get_closest_id, set_id
from std_msgs.msg import Int32, Bool
import traceback


class Tracker:
    def __init__(self) -> None:
        self.model = YOLO('yolov8n.pt')
        image_topic = "/xtion/image_raw"
        self.id_to_follow = 1
        self.last_centroid_x = None
        self._global_frame = "map"
        self._current_image = None
        self.time = rospy.Time.now()
        point_cloud_topic = "/xtion/depth/points"
        self.publish_tf = None
        self._tf_listener = tf.TransformListener()
        self._current_pc = None
        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self._risk_and_direction_pub = rospy.Publisher('/tracker/risk_and_direction', riskanddirection, queue_size=10)
        self._person_to_follow_pub = rospy.Publisher('/tracker/person_to_follow', PointStamped, queue_size=10)
        self._person_detected_pub = rospy.Publisher('/tracker/person_detected', Bool, queue_size=10)
        self._id_detected_pub = rospy.Publisher('/tracker/id_detected', Bool, queue_size=10)

        self.get_closest_id_service = rospy.Service('/tracker/get_closest_id', get_closest_id, self.handler_get_closest_id)
        self.set_id_service = rospy.Service('/tracker/set_id', set_id, self.handler_set_id)

        self.person_detected = False
        self.id_detected = False

        self.ids = []
        self.bboxs = []
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

    def get_closest_id(self, distance):
        print(self.bboxs)
        for index, bbox in enumerate(self.bboxs):
            print(bbox)
            xb, yb = self.calculate_centroid(bbox)
            object_transform = self.calculate_tf(xb, yb)
            print(object_transform)
            if object_transform.point.x < distance:
                return int(self.ids[index])
        return 0

    # These services retake the id, changing it to the close enough person
    def handler_get_closest_id(self, req):
        response = self.get_closest_id(req.dist)
        return response

    # This service changes the id to follow
    def handler_set_id(self, req):
        if req.id is not None and req.id > 0:
            self.id_to_follow = req.id
            return True
        else:
            return False

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
                self._tfpub.sendTransform((object_tf),
                                          tf.transformations.quaternion_from_euler(0, 0, 0),
                                          self.time,
                                          tf_id,
                                          frame)
                self.time = self.time + rospy.Duration(1e-3)
            trans, _ = self.read_published_tf(tf_id, frame)
            if trans:
                point_msg = PointStamped()
                point_msg.header.frame_id = self._global_frame
                point_msg.header.stamp = rospy.Time.now()
                point_msg.point.x = trans[0]
                point_msg.point.y = trans[1]
                point_msg.point.z = 0.0
                self._person_to_follow_pub.publish(point_msg)
                return point_msg

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
        frame_width = 640
        rate = rospy.Rate(30)  # 30 Hz or 30 fps
        while not rospy.is_shutdown():
            if self._current_image is not None:

                small_frame = self._bridge.imgmsg_to_cv2(self._current_image, desired_encoding='bgr8')

                results = self.model.track(source=small_frame, persist=True, classes=0, verbose=False, device=0)
                annotated_frame = results[0].plot()
                for r in results:
                    if hasattr(r, 'boxes') and r.boxes and hasattr(r.boxes, 'id') and r.boxes.id is not None:
                        ids = r.boxes.id.tolist()
                        self.ids = ids
                        bbox_coords_list = r.boxes.xyxy.tolist()
                        self.bboxs = bbox_coords_list

                        for detected_id, bbox_coords in zip(ids, bbox_coords_list):
                            if detected_id == self.id_to_follow:
                                self.person_detected = True
                                self.id_detected = True
                                centroid = self.calculate_centroid(bbox_coords)
                                cx, cy = centroid
                                self.calculate_risk(centroid, frame_width)
                                self.calculate_tf(cx, cy)

                                cv2.circle(annotated_frame, centroid, 5, (0, 255, 0), -1)
                            else:
                                self.person_detected = True
                                self.id_detected = False
                    else:
                        self.person_detected = False
                        self.id_detected = False
                    self._person_detected_pub.publish(self.person_detected)
                    self._id_detected_pub.publish(self.id_detected)

                cv2.imshow("YOLOv8 Tracking", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                rate.sleep()
            else:
                rate.sleep()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('tracker_node', anonymous=True)
    tr = Tracker()
    try:
        tr.main_track()
    except KeyboardInterrupt:
        print("\nEnd of the program :)")
    except rospy.ROSInterruptException:
        pass
