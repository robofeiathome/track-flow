#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener
import tf

class FollowTF(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.target_frame = rospy.get_param('~target_frame', 'moving_target')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        rate = rospy.Rate(10.0)  # Hz
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.base_frame, self.target_frame, rospy.Time(0))
                # Create goal from the received tf
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.base_frame
                goal.target_pose.pose.position.x = trans[0]
                goal.target_pose.pose.position.y = trans[1]
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, tf.transformations.euler_from_quaternion(rot)[2])
                rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
                rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
                self.client.send_goal(goal)
                self.client.wait_for_result()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        return 'success'


def main():
    rospy.init_node('follow_tf')

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('FOLLOW_TF', FollowTF(),
                           transitions={'success':'FOLLOW_TF'})

    outcome = sm.execute()


if __name__ == '__main__':
    main()