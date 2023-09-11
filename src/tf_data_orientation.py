#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler

class DataPublisher:

    def __init__(self):
        # Create listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Create broadcaster
        self.br = TransformBroadcaster()
        # TF frame to follow
        self.frame_id = "person_follow"
        # TF map
        self.target_frame = "map"
        # TF correct frame to follow with orientation
        self.new_frame = "path_to_follow"
        # Previous Transform
        self.previous_tf = None
        # Create a publisher for the transformed data
        self.data_publisher = rospy.Publisher('/path_to_follow', PoseStamped, queue_size=10)
        # Set up timer to call run every 1 second
        self.timer = rospy.Timer(rospy.Duration(1), self.run)

    def lookup_and_correct_tf(self):
        # Read person follow tf
        transform = self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, rospy.Time(0), rospy.Duration(1.0))
        # Change the z to 0.0
        transform.transform.translation.z = 0.0
        return transform

    def adjust_orientation(self, corrected_tf):
        new_transform = None
        if self.previous_tf is not None:
            # Calculate the direction to the new tf and set the rotation to point that direction
            direction = np.array([corrected_tf.transform.translation.x, corrected_tf.transform.translation.y]) - np.array([self.previous_tf.x, self.previous_tf.y])
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
        
        # Publish the transformed data as a message
        if new_transform is not None:
            self.publish_transform_as_message(new_transform)

        # Save this tf for use in the next iteration
        self.previous_tf = corrected_tf.transform.translation
        return new_transform

    def publish_transform_as_message(self, new_transform):
        pose_stamped = PoseStamped()
        pose_stamped.header = new_transform.header
        pose_stamped.pose.position = new_transform.transform.translation
        pose_stamped.pose.orientation = new_transform.transform.rotation
        self.data_publisher.publish(pose_stamped)

    def run(self, event):
        try:
            rospy.loginfo("Before TF lookup")
            # Lookup and correct tf
            corrected_tf = self.lookup_and_correct_tf()
            # Adjust orientation and broadcast new tf
            new_transform = self.adjust_orientation(corrected_tf)
            rospy.loginfo("After TF lookup")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s" % str(e))

if __name__ == "__main__":
    rospy.init_node("tf_data_publisher", log_level=rospy.DEBUG)
    publisher = DataPublisher()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
