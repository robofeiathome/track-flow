#!/usr/bin/env python3

# Author: Mat

import collections
import time
import rospkg
import rospy
import csv
import tf2_ros
from std_msgs.msg import String

# Miscellaneous functions
info = rospy.loginfo
warn = rospy.logwarn
debug = rospy.logdebug
fatal = rospy.logfatal

class data_writter:

    def __init__(self):
        self.file_path = '/home/robofei/Workspace/waypoints.csv'
        # Create listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # TF frame to follow
        self.frame_id = "person_follow"
        self.target_frame = "torso_sensor_plat"
        # Set up timer to call add_tf_as_waypoint every 1 second
        self.timer = rospy.Timer(rospy.Duration(1), self.add_tf_as_waypoint)

    def add_tf_as_waypoint(self, event):
        try:
            rospy.loginfo("Before TF lookup")
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, rospy.Time(0), rospy.Duration(1.0))
            new_waypoint = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
        
            with open(self.file_path, "a") as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(new_waypoint)
                rospy.loginfo("Added Pose")
        
            rospy.loginfo("After TF lookup")
            return new_waypoint
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s" % str(e))
            return None


if __name__ == "__main__":
    rospy.init_node("tf_data", log_level=rospy.DEBUG)
    writer = data_writter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
