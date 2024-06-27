#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from hera_speech.srv import CompareImages, CompareImagesRequest
from hera_tracker.srv import get_closest_id, set_id
from hera_control.srv import Joint_service
from hera_control.msg import Joint_Goal
from hera_tracker.msg import riskanddirection
from hera.msg import moveFeedback, moveResult, moveAction, moveGoal
import traceback
import ast
from std_msgs.msg import Int32, Bool
import sys
import re

import rospkg

directory = rospkg.RosPack().get_path('tasks')
sys.path.append(directory + '/tasks/methods')

# import General as General
# import Speech as Speech
import WhisperSpeech as WhisperSpeech


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
        rospy.wait_for_service('/compareImages')
        rospy.loginfo('compareImages service is ready!')
        self.compare_images = rospy.ServiceProxy('/compareImages', CompareImages)

        #Messages:

        # rospy.wait_for_message('/tracker/risk_and_direction', riskanddirection)
        # rospy.loginfo('risk_and_direction topic is ready!')
        # self.risk_direction = rospy.Subscriber('/tracker/risk_and_direction', riskanddirection, self.callback)
        rospy.wait_for_message('/startOrientation', Bool)
        rospy.loginfo('startOrientation topic is ready!')
        self.start_orientation = rospy.Subscriber('/startOrientation', Bool, self.callbackStartOrientation)
        rospy.wait_for_message('/tracker/id_detected', Bool)
        rospy.loginfo('id_detected topic is ready!')
        self.id_detected = rospy.Subscriber('/tracker/id_detected', Bool, self.callbackPersonDetected)

        #Actions:

        self.client_move = actionlib.SimpleActionClient('move', moveAction)        

        #Variables:
    
        self.speech = WhisperSpeech.WhisperSpeech()
        self.direction = 0.0
        self.MOTOR_POSITION = 0.0
        self.joint_goal = Joint_Goal()
        self.joint_goal.id = 10
        self.joint_goal.x = self.MOTOR_POSITION
        self.poses = [0.0, 0.0, 0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5]
        rospy.loginfo('Ready to Orientate Head!')



    def compare_images_client(self, imgName, contexto):
        rospy.wait_for_service('compareImages')
        try:
            compare_images = rospy.ServiceProxy('compareImages', CompareImages)
            req = CompareImagesRequest()
            req.image_names = imgName #nome da imagem a ser comparada ao frame atual(incluir .jpeg)
            req.context = contexto
            response = compare_images(req)
            print("Response: ", response.message)
            return response.message
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

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
            isonTheFrame = self.compare_images_client('person_follow.jpg', 'Your task is to compare the two images and see if the person on the center of the second image is on the first one (disconsider the id write on the second one, and if the person is in the image return to me the id  with no punctuations and if you cant determine the id return False. If there is no one in the first the person you are loking are not into it so return False as well. Use clothes features and body features, and if it is the same person return the id.Attention: Make sure is the same person, use skin color, hair color, clothes color and type, body features, clothes features to determine, be extremely precise with it as you will set the id for the person that the robot needs to follow (the images are taken with minutes of diference, the person will be using the same clothes). Return to me just the ID or False, bowth with no punctuations please. BE EXTREMELY PRECISE WITH THE COMPARISION IF YOU DO NOT FIND THE RIGHT PERSON JUST RETURN False')
            print(isonTheFrame, type(isonTheFrame))
            try:
                # Use a regular expression to find the ID or "False"
                match = re.search(r'\d+|False', isonTheFrame)
                if match:
                    result = match.group(0)
                    if result == 'False':
                        newID = False
                    else:
                        newID = int(result)
                else:
                    newID = False
            except Exception as e:
                print(traceback.format_exc())
                newID = False
            if newID is not None:
                return newID
        except Exception as e:
            print(traceback.format_exc())

        return False

    
    def move(self, command: str, velocity: float = 0.2, duration: float = 0.0):
        goal = moveGoal(cmd=command, vel=velocity, seconds=duration)
        self.client_move.send_goal(goal)
        self.client_move.wait_for_result()
        return self.client_move.get_result()


    def move_head(self, direction):

        self.joint_goal.x = direction

        self.manipulator(type='', goal=self.joint_goal)

    def callbackStartOrientation(self, data):
        try:
            if data == None:
                self.start_orientation = False
            else:
                self.start_orientation = data.data
        except Exception as e:
            print(traceback.format_exc())

    def orientate(self):
        if not self.id_detected:
                self.speech.talk('I lost you, can you please wait for me to find you?')
                for i in range (len(self.poses)):
                    self.move_head(self.poses[i])
                    newID = self.lookfor()
                    print(newID, type(newID))
                    rospy.sleep(2)
                    if newID:
                        self.set_id(newID)
                        self.speech.talk('I found you! I will follow you now!')
                        self.id_detected = True
                        break
                    else:
                        rospy.sleep(3)

    def main(self):
        while not rospy.is_shutdown():
            if self.start_orientation == True:
                self.orientate()

if __name__ == "__main__":
    rospy.init_node('RecoveryNode', anonymous=True)
    tr = RecoveryMode()
    try:
        tr.main()
    except KeyboardInterrupt:
        print("\nEnd of the program :)")
    except rospy.ROSInterruptException:
        pass
