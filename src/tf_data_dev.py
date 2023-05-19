#!/usr/bin/env python3

# Author: Mat e leled

import rospy
import csv
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose


class DataWriter:

    def __init__(self):
        self.file_path = '/home/robofei/Workspace/waypoints.csv'
        # Create listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Create broadcaster
        self.br = TransformBroadcaster()
        # TF frame to follow
        self.frame_id = "person_follow"
        # TF map
        self.target_frame = "map"
        # TF correct frame to follow
        self.new_frame = "path_to_follow"
        # Previous Transform
        self.previous_tf = None
        self.previous_position = []
        # Set up timer to call add_tf_as_waypoint every 1 second
        self.timer = rospy.Timer(rospy.Duration(1), self.add_tf_as_waypoint)

    def add_tf_as_waypoint(self, event):
        try:
            rospy.loginfo("Before TF lookup")
            
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, rospy.Time(0), rospy.Duration(1.0))
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Change the z to 0.0 and publish the new transform
            trans.z = 0.0
            new_transform = TransformStamped()
            new_transform.header.stamp = rospy.Time.now()
            new_transform.header.frame_id = self.frame_id
            new_transform.child_frame_id = self.new_frame
            new_transform.transform.translation = trans
            new_transform.transform.rotation = rot
            self.br.sendTransformMessage(new_transform)

            # Transform the new transform to map frame
            transformed = self.tf_buffer.transform(new_transform, self.target_frame)
            transmed = transformed.transform.translation
            roted = transformed.transform.rotation


            if self.previous_tf is not None:
                direction = np.array([transmed.x, transmed.y]) - np.array([self.previous_tf.x, self.previous_tf.y])
                angle = np.arctan2(direction[1], direction[0])
                q = quaternion_from_euler(0, 0, angle)
                transmed.x = self.previous_tf.x
                transmed.y = self.previous_tf.y
                roted.x = 0.0
                roted.y = 0.0
                roted.z = q[2]
                roted.w = q[3]
                # Save this tf for use in the next iteration
                self.previous_tf = transformed
        
                pose = PoseStamped()
                pose.pose.position = transformed.transform.translation
                pose.pose.orientation = transformed.transform.rotation

                new_waypoint = [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w
                ]

                with open(self.file_path, "a") as csv_file:
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow(new_waypoint)
                    rospy.loginfo("Added Pose")
            
                rospy.loginfo("After TF lookup")
                return new_waypoint
            else:
                self.previous_tf= transformed
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s" % str(e))
            return None


if __name__ == "__main__":
    rospy.init_node("tf_data", log_level=rospy.DEBUG)
    writer = DataWriter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
