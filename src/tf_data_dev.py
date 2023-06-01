#!/usr/bin/env python3

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
        # TF corrected frame to follow
        self.corrected_frame = "xyzcorrect_follow"
        # TF correct frame to follow with orientation
        self.new_frame = "path_to_follow"
        # Previous Transform
        self.previous_tf = None
        # Set up timer to call run every 1 second
        self.timer = rospy.Timer(rospy.Duration(1), self.run)

    def lookup_and_correct_tf(self):
        # Read person follow tf
        transform = self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, rospy.Time(0), rospy.Duration(1.0))
        # Change the z to 0.0
        transform.transform.translation.z = 0.0
        return transform

    def adjust_orientation(self, corrected_tf):
        if self.previous_tf is not None:
            # Calculate the direction to the new tf and set the rotation to point that direction
            direction = np.array([corrected_tf.transform.translation.x, corrected_tf.transform.translation.y]) - np.array([self.previous_tf.x, self.previous_tf.y])
            
            # Check if x and y are the same as before
            if np.allclose(direction, 0.0):
                return None
            
            angle = np.arctan2(direction[1], direction[0])
            q = quaternion_from_euler(0, 0, angle)
            corrected_tf.transform.rotation.x = 0.0
            corrected_tf.transform.rotation.y = 0.0
            corrected_tf.transform.rotation.z = q[2]
            corrected_tf.transform.rotation.w = q[3]

            # Broadcast tf with correct rotation
            new_transform = TransformStamped()
            new_transform.header.stamp = rospy.Time.now()
            new_transform.header.frame_id = self.target_frame
            new_transform.child_frame_id = self.new_frame
            new_transform.transform = corrected_tf.transform
            self.br.sendTransform(new_transform)
            return new_transform

        # Save this tf for use in the next iteration
        self.previous_tf = corrected_tf.transform.translation
        return None

    def write_tf_to_csv(self, new_transform):
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

    def run(self, event):
        try:
            rospy.loginfo("Before TF lookup")
            # Lookup and correct tf
            corrected_tf = self.lookup_and_correct_tf()
            # Adjust orientation and broadcast new tf
            new_transform = self.adjust_orientation(corrected_tf)
            # Write the new transform to CSV if it's not the first one and the x and y have changed
            if new_transform is not None:
                self.write_tf_to_csv(new_transform)
            rospy.loginfo("After TF lookup")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s" % str(e))


if __name__ == "__main__":
    rospy.init_node("tf_data", log_level=rospy.DEBUG)
    writer = DataWriter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
