import cv2
from ultralytics import YOLO
import rospy
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2

class tracker:
    def __init__(self) -> None:   
        self.model = YOLO('yolov8n.pt')
        self.source = 0
        self.id_to_follow = 1
        self._global_frame = 'zed2i_left_camera_frame'
        self.time=rospy.Time.now()
        point_cloud_topic = '/zed_node/point_cloud/cloud_registered'
        self.publish_tf=None
        self._tf_listener = tf.TransformListener()
        self._current_pc = None
        self._bridge = CvBridge()

        # Publisher for frames with detected objects
        self._imagepub = rospy.Publisher('~objects_label', Image, queue_size=10)
        if point_cloud_topic is not None:
            rospy.Subscriber(point_cloud_topic, PointCloud2, self.pc_callback)
        else:
            rospy.loginfo('No point cloud information available. Objects will not be placed in the scene.')

        self._tfpub = tf.TransformBroadcaster()
        rospy.loginfo('Ready to Track!')

    def pc_callback(self, pc):
        """Point cloud callback"""
        # Store value on a private attribute
        self._current_pc = pc

    # This function uses the centroid of the bbox to calculate the tf
    def calculate_tf (self, current_centroid_x, current_centroid_y):

        (trans, _) = self._tf_listener.lookupTransform('/' + self._global_frame, '/zed2i_camera_center', rospy.Time(0))

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

        if self.publish_tf:
            # Object tf (x, y, z) must be passed as (z, -x, -y)
            object_tf = [point_z, point_x, point_y]
            frame = 'zed2i_camera_center'

            # Translate the tf in regard to the fixed frame
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

    #This function calculates the centroids of the bbox
    def calculate_centroid(self, bbox):
        x_min, y_min, x_max, y_max = bbox
        return int((x_min + x_max) / 2), int((y_min + y_max) / 2)

    #This function performes some things, object detection, object tracker and then filter it by its id and class
    def main_track(self):
        cap = cv2.VideoCapture(self.source)
        rate = rospy.Rate(30)  # 30 Hz or 30 fps
        while cap.isOpened():
            success, frame = cap.read()
            if success:
                #This line alone is responsible for the detection and track (YOLOv8 + BOTsort)
                results = self.model.track(frame, persist=True, classes=0, verbose=False, device=0)
                annotated_frame = results[0].plot()
                
                for r in results:
                    #This will check the ids and coordboxes
                    if hasattr(r, 'boxes') and r.boxes and hasattr(r.boxes, 'id') and r.boxes.id is not None:
                        ids = r.boxes.id.tolist()
                        bbox_coords_list = r.boxes.xyxy.tolist()

                        for detected_id, bbox_coords in zip(ids, bbox_coords_list):
                            if detected_id == self.id_to_follow:
                                centroid = self.calculate_centroid(bbox_coords)
                                cx, cy = centroid
                                self.calculate_tf(cx,cy)
                                print(f"Centroid for ID {self.id_to_follow}:", centroid)
                                cv2.circle(annotated_frame, centroid, 5, (0, 255, 0), -1)

                cv2.imshow("YOLOv8 Tracking", annotated_frame)
                
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                rate.sleep()  # Ensure the loop runs at 30 fps

            else:
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":    
    rospy.init_node('tracker_node', anonymous=True)
    try:
        tracker().main_track()
    except KeyboardInterrupt:
        print("\nEnd of the program :)")
    except rospy.ROSInterruptException:
        pass
