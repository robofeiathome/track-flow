#!/usr/bin/env python3

# Authors: Mat e leled

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
        # Set up timer to call add_tf_as_waypoint every 1 second
        self.timer = rospy.Timer(rospy.Duration(1), self.add_tf_as_waypoint)

    def add_tf_as_waypoint(self, event):
        try:
            rospy.loginfo("Before TF lookup")

            transform = self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, rospy.Time(0), rospy.Duration(1.0))
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Change the z to 0.0
            
            trans.z = 0.0

            # If previous_tf exists, calculate the direction to the new tf and set the rotation to point that direction
            if self.previous_tf is not None:
                direction = np.array([trans.x, trans.y]) - np.array([self.previous_tf.x, self.previous_tf.y])
                angle = np.arctan2(direction[1], direction[0])
                q = quaternion_from_euler(0, 0, angle)
                # trans.x = self.previous_tf.x
                # trans.y = self.previous_tf.y
                rot.x = 0.0
                rot.y = 0.0
                rot.z = q[2]
                rot.w = q[3]

            # Save this tf for use in the next iteration
            self.previous_tf = trans

            new_transform = TransformStamped()
            new_transform.header.stamp = rospy.Time.now()
            new_transform.header.frame_id = self.target_frame
            new_transform.child_frame_id = self.new_frame
            new_transform.transform.translation = trans
            new_transform.transform.rotation = rot
            self.br.sendTransform(new_transform)

            # Create a PoseStamped from the TransformStamped
            pose_stamped = PoseStamped()
            pose_stamped.header = new_transform.header
            pose_stamped.pose.position = new_transform.transform.translation
            pose_stamped.pose.orientation = new_transform.transform.rotation

            # Fetch the transform between the frames
            trans = self.tf_buffer.lookup_transform(self.target_frame, new_transform.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Manually apply the transformation to the pose
            transformed = do_transform_pose(pose_stamped, trans)

            new_waypoint = [
                transformed.pose.position.x,
                transformed.pose.position.y,
                transformed.pose.position.z,
                transformed.pose.orientation.x,
                transformed.pose.orientation.y,
                transformed.pose.orientation.z,
                transformed.pose.orientation.w
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
    writer = DataWriter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
