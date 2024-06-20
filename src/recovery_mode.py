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
        rospy.wait_for_service('/compareImages')
        rospy.loginfo('compareImages service is ready!')
        self.compare_images = rospy.ServiceProxy('/compareImages', CompareImages)

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



    def compare_images_client(self, imgName, contexto):
        rospy.wait_for_service('compareImages')
        try:
            compare_images = rospy.ServiceProxy('compareImages', CompareImages)
            req = CompareImagesRequest()
            req.image_names = imgName #nome da imagem a ser comparada ao frame atual(incluir .jpeg)
            req.context = contexto
            response = compare_images(req)
            print("Response: ", response.message)
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
        direction = int(self.direction.data)
        for i in range(3):
            try:
                if not self.id_detected:
                    isonTheFrame = self.compare_images_client('person_follow', 'sua tarefa e compara as duas imagens e determinar se a pessoa da segunda imagem se encontra na primeira imagem, seja muito cauteloso, use detalhes da roupa, acessorios, cor da pele, cabelo, etc. O robo esta performando uma tarefa de carry my luggage, por favor, ajude-o a encontrar a pessoa que ele deve seguir. Retorne para mim apenas "(ID)"(onde id e a id que aparece para a pessoa na imagem) ou "nao".')
                    try:
                        newID = ast.literal_eval(isonTheFrame)
                    except:
                        newID = False
                    if newID:
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

    def main(self):
        while not rospy.is_shutdown():
            if not self.id_detected:
                newID = self.lookfor()
                if newID:
                    self.set_id(newID)
                else:
                    if self.direction>0:
                        self.move_head(-1.5)
                    else:
                        self.move_head(1.5)


if __name__ == "__main__":
    rospy.init_node('RecoveryNode', anonymous=True)
    tr = RecoveryMode()
    try:
        tr.main()
    except KeyboardInterrupt:
        print("\nEnd of the program :)")
    except rospy.ROSInterruptException:
        pass
