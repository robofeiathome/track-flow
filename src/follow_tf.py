#!/usr/bin/env python3

# Author: Nik and GPT

import collections
import time
import rospkg
import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener
from follow_me.srv import FollowWaypointsService
from std_msgs.msg import String

# Miscellaneous functions
info = rospy.loginfo
warn = rospy.logwarn
debug = rospy.logdebug
fatal = rospy.logfatal

class Follow:
    """
    Read goal poses/locations from TF, instruct move_base to follow them.
    """

    def __init__(self):
        rospy.Service("follow_service", FollowWaypointsService, self.handler)

        self.waypoints = collections.deque()

        self.frame_id =  "map"
        self.wait_duration =  0.2
        self.actual_xy_goal_tolerance = 0.4
        self.actual_yaw_goal_tolerance =  3.14
        self.distance_tolerance =  0.2

        self.tf_listener = TransformListener()

        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        self.stop_follow = False

        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        info("Connecting to move_base...")
        self.move_base_client.wait_for_server()
        info("Connected to move_base.")

        self.last_xy_goal_tolerance = rospy.get_param("/move_base/DWAPlannerROS/xy_goal_tolerance")
        self.last_yaw_goal_tolerance = rospy.get_param("/move_base/DWAPlannerROS/yaw_goal_tolerance")

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.loss = rospy.Publisher('Cancel_waypoints', String, queue_size=10)

        self.rate.sleep()

    def send_move_base_goal(self, poses):
        """Assemble and send a new goal to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position = poses.pose.position
        goal.target_pose.pose.orientation = poses.pose.orientation
        info(
            "Executing move_base goal -> (%s, %s) ..."
            % (poses.pose.position.x, poses.pose.position.y)
        )

        self.move_base_client.send_goal(goal)

    def run_once(self):
        """Single iteration of the node loop."""
        if len(self.waypoints) == 0:
            warn("Loop Follow Start")
            self.add_tf_as_waypoint()

        while self.waypoints and not rospy.is_shutdown() and not self.stop_follow:
            goal = self.waypoints.popleft()

            self.send_move_base_goal(goal)

            self.move_base_client.wait_for_result()
            info(
                "Waiting for %f seconds before proceeding to the next goal..."
                % self.wait_duration
            )

            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                info("Goal successfully reache: %s" % str(goal))
                time.sleep(self.wait_duration)
            else:
                cancel = str(goal)
                self.loss.publish(cancel)
                warn("Failed to reach goal: %s" % str(goal))

            time.sleep(self.wait_duration)

    def add_tf_as_waypoint(self):
        try:
            now = rospy.Time(0)
            self.tf_listener.waitForTransform("/person_follow", "/map", now, rospy.Duration(1.0))
            (trans,rot) = self.tf_listener.lookupTransform("/person_follow", "/map", now)
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            self.waypoints.append(pose)
            rospy.sleep(1)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            fatal("Failed to get transform")
            return False

    def run(self):
        info("run ...")
        while not rospy.is_shutdown():
            self.run_once()

    def stop(self):
        self.client.wait_for_server()
        self.client.cancel_all_goals()
        rospy.loginfo("Stop Follow")

    def handler(self, request):
        if request.command == "start":
            self.run()
            return "Started following waypoints"

        if request.command == "stop":
            self.stop_follow = True
            self.stop()
            return "Stopped following waypoints"

        elif request.command == "restart":
            self.stop_follow = False
            return "Restart following waypoints"
        else:
            return "Unknown command"


if __name__ == "__main__":
    rospy.init_node("follow", log_level=rospy.DEBUG)
    Follow()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
