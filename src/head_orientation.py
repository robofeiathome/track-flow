#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from hera_control.srv import Joint_service
from hera_control.msg import Joint_Goal
from hera_tracker.msg import riskanddirection
import traceback
from std_msgs.msg import Int32, Bool


class HeadOrientation:
    def __init__(self) -> None:
        rospy.wait_for_service('/joint_command')
        self.manipulator = rospy.ServiceProxy('/joint_command', Joint_service)
        rospy.wait_for_message('/tracker/risk_and_direction', riskanddirection)
        self.risk_direction = rospy.Subscriber('/tracker/risk_and_direction', riskanddirection, self.callback)
        rospy.wait_for_message('/tracker/person_detected', Bool)
        self.person_detected = rospy.Subscriber('/tracker/person_detected', Bool, self.callbackPersonDetected)

        self.direction = 0.0
        self.MOTOR_POSITION = 0.0
        self.joint_goal = Joint_Goal()
        self.joint_goal.id = 10
        self.joint_goal.x = self.MOTOR_POSITION
        rospy.loginfo('Ready to Orientate Head!')

    def callbackPersonDetected(self, data):
        try:
            if data == None:
                self.person_detected = False
            else:
                self.person_detected = data.data
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

    def move_head(self, direction):
        print(self.person_detected)
        if self.person_detected:
            c_direction = int(direction.data)
            if 1 < c_direction < 6 and self.MOTOR_POSITION >= -1.8:
                self.MOTOR_POSITION -= 0.2
            elif 6 < c_direction < 8 and self.MOTOR_POSITION >= -1.8:
                self.MOTOR_POSITION -= 0.4
            elif -6 < c_direction < -1 and self.MOTOR_POSITION <= 1.8:
                self.MOTOR_POSITION += 0.2
            elif -8 < c_direction < -6 and self.MOTOR_POSITION <= 1.8:
                self.MOTOR_POSITION += 0.4
            elif -1 < c_direction < 1:
                pass
        else:
            self.MOTOR_POSITION = 0.0

        self.joint_goal.x = self.MOTOR_POSITION

        self.manipulator(type='', goal=self.joint_goal)

    def main(self):
        self.manipulator(type='', goal=self.joint_goal)
        while not rospy.is_shutdown():
            self.move_head(self.direction)


if __name__ == "__main__":
    rospy.init_node('headOrientation_node', anonymous=True)
    tr = HeadOrientation()
    try:
        tr.main()
    except KeyboardInterrupt:
        print("\nEnd of the program :)")
    except rospy.ROSInterruptException:
        pass
