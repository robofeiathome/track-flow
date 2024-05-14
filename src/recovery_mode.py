#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from hera_tracker.srv import get_closest_id, set_id
from hera_control.srv import Joint_service
from hera_control.msg import Joint_Goal
from hera_tracker.msg import riskanddirection
from hera.msg import moveFeedback, moveResult, moveAction, moveGoal
import traceback
from std_msgs.msg import Int32, Bool

#This is a stupid implementation

class RecoveryMode:
    def __init__(self) -> None:

        #Services:

        rospy.wait_for_service('/joint_command')
        rospy.loginfo('joint_command service is ready!')
        self.manipulator = rospy.ServiceProxy('/joint_command', Joint_service)
        rospy.wait_for_service('/tracker/get_closest_id')
        rospy.loginfo('get_closest_id service is ready!')
        self.get_closest_id = rospy.ServiceProxy('/tracker/get_closest_id', get_closest_id)
        rospy.wait_for_service('/tracker/set_id')
        rospy.loginfo('set_id service is ready!')
        self.set_id = rospy.ServiceProxy('/tracker/set_id', set_id)

        #Messages:

        rospy.wait_for_message('/tracker/risk_and_direction', riskanddirection)
        rospy.loginfo('risk_and_direction topic is ready!')
        self.risk_direction = rospy.Subscriber('/tracker/risk_and_direction', riskanddirection, self.callback)
        rospy.wait_for_message('/tracker/id_detected', Bool)
        rospy.loginfo('id_detected topic is ready!')
        self.id_detected = rospy.Subscriber('/tracker/id_detected', Bool, self.callbackPersonDetected)

        #Actions:

        self.client_move = actionlib.SimpleActionClient('move', moveAction)        

        #Variables:
    
        self.direction = 0.0
        self.MOTOR_POSITION = 0.0
        self.joint_goal = Joint_Goal()
        self.joint_goal.id = 10
        self.joint_goal.x = self.MOTOR_POSITION
        rospy.loginfo('Ready to Orientate Head!')

    def callbackPersonDetected(self, data):
        try:
            if data == None:
                self.id_detected = False
            else:
                self.id_detected = data.data
        except Exception as e:
            print(traceback.format_exc())

    def callback(self, data):
        try:
            if data == None:
                self.direction = self.direction
            else:
                self.direction = data.risk
        except Exception as e:
            print(traceback.format_exc())

    def lookfor(self):
        try:
            if self.direction < 0:
                self.move_head(1.8)
            elif self.direction > 0:
                self.move_head(-1.8)
            closest_id = self.get_closest_id(3).id
            if closest_id != 0:
                self.set_id(closest_id)
            else:
                if self.direction < 0:
                    self.move('spin_left', 0.4, 1)

                elif self.direction > 0:
                    self.move('spin_right', 0.4, 1)
                else:
                    pass
        except Exception as e:
            print(traceback.format_exc())

    
    def move(self, command: str, velocity: float = 0.2, duration: float = 0.0):
        goal = moveGoal(cmd=command, vel=velocity, seconds=duration)
        self.client_move.send_goal(goal)
        self.client_move.wait_for_result()
        return self.client_move.get_result()


    def move_head(self, direction):

        self.joint_goal.x = direction

        self.manipulator(type='', goal=self.joint_goal)

    def main(self):
        while not rospy.is_shutdown():
            if not self.id_detected:
                self.lookfor()


if __name__ == "__main__":
    rospy.init_node('RecoveryNode', anonymous=True)
    tr = RecoveryMode()
    try:
        tr.main()
    except KeyboardInterrupt:
        print("\nEnd of the program :)")
    except rospy.ROSInterruptException:
        pass
