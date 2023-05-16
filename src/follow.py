#!/usr/bin/env python3

# Author: Nik

import collections
import time
import rospkg
import actionlib
import rospy
import csv
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros

from follow_me.srv import FollowWaypointsService
from std_msgs.msg import String

# Miscellaneous functions
info = rospy.loginfo
warn = rospy.logwarn
debug = rospy.logdebug
fatal = rospy.logfatal

class Follow:
    """
    Read goal poses/locations from the designated topics, instruct move_base to follow them.
    """

    def __init__(self):
        rospy.Service("follow_service", FollowWaypointsService, self.handler)

        self.waypoints = collections.deque()

        # Create listener
#        self.tf_buffer = tf2_ros.Buffer()
#        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # TF frame to follow
#        self.target_frame = "base_footprint"

        # Read parameters off the parameter server --------------------------------------------
        self.index = 1
        self.frame_id = "map"
        self.wait_duration = 0.2
        self.actual_xy_goal_tolerance = 0.4
        self.actual_yaw_goal_tolerance = 3.14
        self.distance_tolerance = 0.0
        self.file_path = '/home/robofei/Workspace/catkin_ws/src/3rd_party/Vision_System/track-flow/src/follow_utils/waypoints.csv'

        # Is the path provided by the user ready to follow?
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        self.coord = []
        self.coord_all = []

        self.init_pose(self.file_path)
        self.add_waypoints(self.coord, self.waypoints)

        # move_base Action client -------------------------------------------------------------
        self.move_base_client = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction
        )
        info("Connecting to move_base...")
        self.move_base_client.wait_for_server()
        info("Connected to move_base.")
        self.stop_follow = False

        self.last_xy_goal_tolerance = rospy.get_param(
            "/move_base/DWAPlannerROS/xy_goal_tolerance"
        )
        self.last_yaw_goal_tolerance = rospy.get_param(
            "/move_base/DWAPlannerROS/yaw_goal_tolerance"
        )
        # create an action client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.loss = rospy.Publisher('Cancel_waypoints', String, queue_size=10)

        self.rate.sleep()

    # def add_tf_as_waypoint(self):
    #     try:
    #         transform = self.tf_buffer.lookup_transform(self.frame_id, self.target_frame, rospy.Time(0), rospy.Duration(1.0))
    #         new_waypoint = [
    #             transform.transform.translation.x,
    #             transform.transform.translation.y,
    #             transform.transform.translation.z,
    #             transform.transform.rotation.x,
    #             transform.transform.rotation.y,
    #             transform.transform.rotation.z,
    #             transform.transform.rotation.w
    #         ]
            
    #         with open(self.file_path, "a") as csv_file:
    #             csv_writer = csv.writer(csv_file)
    #             csv_writer.writerow(new_waypoint)
            
    #         return new_waypoint
        
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         warn("TF lookup failed: %s" % str(e))
    #         return None
        
    # helper methods
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
        self.add_waypoints(self.coord, self.waypoints)

        if len(self.coord) == 0:
            warn("Loop Follow Start")
            self.add_waypoints(self.coord_all, self.waypoints)

        # we have waypoints, let's follow them!
        while self.waypoints and not rospy.is_shutdown() and not self.stop_follow:
            goal = self.waypoints[0]
            
            self.waypoints.popleft()

            self.send_move_base_goal(goal)

            if not self.distance_tolerance > 0.0:
                # just wait until move_base reaches the goal...
                self.move_base_client.wait_for_result()
                info(
                    "Waiting for %f seconds before proceeding to the next goal..."
                    % self.wait_duration
                )

                state = self.move_base_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    info("Goal successfully reache: %s" % str(goal))
                    self.coord.pop(0)
                    time.sleeop(self.wait_duration)
                else:
                    cancel = str(goal)
                    self.loss.publish(cancel)
                    warn("Failed to reach goal: %s" % str(goal))

                self.coord.pop(0)
                time.sleep(self.wait_duration)

            else:
                raise NotImplementedError("distance_tolerance not implemented yet.")

    # helper methods
    def add_waypoints(self, coord, waypoints):
        try:
            for row in coord:
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = float(row[0])
                pose.pose.position.y = float(row[1])
                pose.pose.position.z = float(row[2])
                pose.pose.orientation.x = float(row[3])
                pose.pose.orientation.y = float(row[4])
                pose.pose.orientation.z = float(row[5])
                pose.pose.orientation.w = float(row[6])
                waypoints.append(pose)
            return True
        except:
            fatal("Failed to add waypoints")
            return False

    def init_pose(self, file_path):
        with open(file_path, "r") as csv_file:
            reader = csv.reader(csv_file)
            for i, row in enumerate(reader):
                self.coord_all.append(row)
                if self.index <= i:
                    self.coord.append(row)

    def run(self):
        info("run ...")
        while not rospy.is_shutdown():
            #new_waypoint = self.add_waypoints()
            #if new_waypoint:
            #    self.coord.append(new_waypoint)
            #    self.coord_all.append(new_waypoint)
            self.run_once()

    def stop(self):
        # wait for the action server to start up
        self.client.wait_for_server()

        # cancel all goals sent by this action client
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

