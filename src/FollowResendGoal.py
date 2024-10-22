#!/usr/bin/env python3

import rospy
import sys
import json
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from hera_tracker.srv import get_closest_id, set_id, follow
from std_msgs.msg import Bool
import time
import tf2_ros
import tf
import numpy as np
import shutil
import os

# Adiciona o caminho dos métodos do pacote de tarefas
path = rospkg.RosPack().get_path('tasks')
sys.path.append(path + '/tasks/methods')

# Importa módulos personalizados
import General as General
import WhisperSpeech as WhisperSpeech
import Perception as Perception
import Navigation as Navigation
import Manipulator as Manipulator

# Alias para funções de logging
info = rospy.loginfo
warn = rospy.logwarn
err = rospy.logerr

### Author: Mateus Scarpelli
### Credits to: Khaled Hazime (Navigation methods created by him)

class NavigationAndFollow:
    """Classe para a tarefa de navegação e acompanhamento do RoboFEI LARC/CBR 2023"""

    def __init__(self):

        self.id_detected = None
        self.person_detected = None
        self.perception = Perception.Perception()
        self.navigation = Navigation.Navigation()
        self.manipulator = Manipulator.Manipulator()
        self.speech = WhisperSpeech.WhisperSpeech()
        self.general = General.General()
        
        rospy.Service('follow', follow, self.follow_handler)

        # Cria o listener TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Define o frame TF a seguir
        self.frame_id = "person_follow"
        # Define o frame de referência
        self.target_frame = "map"

        # Subscreve aos tópicos relevantes
        self.id_detected_sub = rospy.Subscriber('/tracker/id_detected', Bool, self.id_detected_callback)
        self.person_detected_sub = rospy.Subscriber('/tracker/person_detected', Bool, self.person_detected_callback)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.set_id_srv = rospy.ServiceProxy('/tracker/set_id', set_id)
        self.get_closest_id_srv = rospy.ServiceProxy('/tracker/get_closest_id', get_closest_id)
        self.last_goal_pos = None
        self.last_goal_rot = None
        self.locals = ['point_1', 'point_2', 'exit']
        self._tfpub = tf.TransformBroadcaster()
        self.msg = False
        self.possible_answers = ['yes', 'no']	

        self._start_orientation_pub = rospy.Publisher('/startOrientation', Bool, queue_size=10)
        

        self.last_position = None  # Adiciona variável para armazenar a última posição
        self.distance_checks = 0  # Contador para checagens de distância

    def determine_person_position(self):
        """Obtém a transformação do frame da pessoa a seguir"""
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, rospy.Time(0), rospy.Duration(1.0))
            # Altera a coordenada z para 0.0
            transform.transform.translation.z = 0.0

            # Extrai a posição e rotação
            pos = transform.transform.translation
            rot = transform.transform.rotation

            # Cria tuplas para posição e rotação
            position_tuple = (pos.x, pos.y, pos.z)
            rotation_tuple = (rot.x, rot.y, rot.z, rot.w)

            # Filtra a nova posição com base na distância da última posição

            # Log da transformação
            rospy.loginfo(f"Transform - Position: {position_tuple}, Rotation: {rotation_tuple}")

            # Republishes the modified transformation
            self._tfpub.sendTransform(position_tuple, rotation_tuple, rospy.Time.now(), "person_ground", self.target_frame)

            return pos, rot

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            return None, None



    def person_tf(self):
        """Obtém a transformação do frame da pessoa a seguir"""
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, rospy.Time(0), rospy.Duration(1.0))
            # Altera a coordenada z para 0.0
            transform.transform.translation.z = 0.0

            # Extrai a posição e rotação
            pos = transform.transform.translation

            # Check if the point is on the costmap
            rot = transform.transform.rotation

            # Cria tuplas para posição e rotação
            position_tuple = (pos.x, pos.y, pos.z)
            rotation_tuple = (rot.x, rot.y, rot.z, rot.w)

            # Filtra a nova posição com base na distância da última posição
            if self.last_position:
                distance = np.sqrt((pos.x - self.last_position.x) ** 2 + (pos.y - self.last_position.y) ** 2)
                if distance > 3.0:
                    self.distance_checks += 1
                    rospy.logwarn(f"Distance check {self.distance_checks}/5: {distance} meters")
                    if self.distance_checks >= 5:
                        self.distance_checks = 0
                        self.last_position = Point(pos.x, pos.y, pos.z)
                        rospy.loginfo(f"Accepting position due to consistent distance check: {position_tuple}")
                        return pos, rot
                    return None, None
                else:
                    self.distance_checks = 0

            # Atualiza a última posição
            #self.last_position = Point(pos.x, pos.y, pos.z)

            # Log da transformação
            rospy.loginfo(f"Transform - Position: {position_tuple}, Rotation: {rotation_tuple}")

            # Republishes the modified transformation
            self._tfpub.sendTransform(position_tuple, rotation_tuple, rospy.Time.now(), "person_ground", self.target_frame)

            return pos, rot

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            return None, None

    def normalize_quaternion(self, q):
        """Normaliza um quaternion para o comprimento unitário."""
        try:
            norm = np.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            rospy.loginfo(f"Quaternion before normalization: ({q.x}, {q.y}, {q.z}, {q.w}), norm: {norm}")
            
            if norm == 0:
                raise ValueError("Quaternion norm is zero, cannot normalize.")

            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm
            rospy.loginfo(f"Quaternion after normalization: ({q.x}, {q.y}, {q.z}, {q.w})")
            return q
        except Exception as e:
            rospy.logerr(f"Error normalizing quaternion: {e}")
            raise

    def calculate_orientation(self, from_point, to_point):
        """Calcula a orientação como um quaternion (apontando para o ponto de destino)."""
        dx = to_point.x - from_point.x
        dy = to_point.y - from_point.y
        yaw = np.arctan2(dy, dx)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(*quaternion)

    def id_detected_callback(self, msg):
        """Callback para o tópico /tracker/id_detected"""
        self.id_detected = msg.data

    def person_detected_callback(self, msg):
        """Callback para o tópico /tracker/person_detected"""
        self.person_detected = msg.data

    def send_goal_to_move_base(self, position, orientation, wait_for_result=False):
        """Envia um objetivo para o move_base"""
        new_pose = PoseStamped()
        new_pose.header.frame_id = 'map'
        new_pose.header.stamp = rospy.Time.now()
        new_pose.pose.position.x = (position.x)/2
        new_pose.pose.position.y = (position.y)
        new_pose.pose.position.z = 0.0

        try:
            # Normaliza e define a orientação
            normalized_orientation = self.normalize_quaternion(orientation)
            new_pose.pose.orientation = normalized_orientation

            # Log da nova pose
            rospy.loginfo(f"New Pose - Position: {new_pose.pose.position}, Orientation: {new_pose.pose.orientation}")

            goal = MoveBaseGoal()
            goal.target_pose = new_pose

            # Verifica se o objetivo é válido
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'map'
            start_pose.header.stamp = rospy.Time.now()
            start_pose.pose.position = self.last_position
            start_pose.pose.orientation.w = 1.0
            try:
                self.move_base_client.send_goal(goal)
                rospy.loginfo("Goal sent to move_base")
            except Exception as e:
                rospy.logerr(f"Failed to send goal to move_base: {e}")

            if wait_for_result:
                try:
                    self.move_base_client.wait_for_result()
                    result = self.move_base_client.get_result()
                    rospy.loginfo(f"Result from move_base: {result}")
                    return result
                except Exception as e:
                    rospy.logerr(f"Error waiting for result from move_base: {e}")
        
        except Exception as e:
            rospy.logerr(f"Failed to normalize orientation or send goal: {e}")

    def check_for_movement(self, position):
        """Verifica se houve movimento entre a última posição e a atual."""
        if self.last_goal_pos:
            distance = np.sqrt((position.x - self.last_goal_pos.x) ** 2 + (position.y - self.last_goal_pos.y) ** 2)
            rospy.loginfo(f"Distance from last goal: {distance} meters")
            if distance < 0.2:
                rospy.loginfo("No significant movement detected.")
                return False
            else:
                rospy.loginfo("Significant movement detected.")
                return True
        else:
            rospy.loginfo("No last goal position to compare.")
            return True
        
    def main(self):
        """Método principal para a tarefa de navegação e acompanhamento"""
        self.msg = True
        rospy.loginfo('Starting Navigation and Follow task')

        while not self.perception.face_check(True):
            pass

        id = self.get_closest_id_srv(3).id
        self.set_id_srv(id)
        rospack = rospkg.RosPack()
        self.perception.save_face('person_follow')

        new_path = rospack.get_path('hera_speech') + '/src/images/' + 'person_follow.jpg'
        source = rospack.get_path('hera_face') + '/face_images/' + 'person_follow.jpg'
        shutil.move(source, new_path)

        if self.id_detected:
            counter_for_stop = 0
            for _ in range(3):
                self._start_orientation_pub.publish(self.msg)

            self.last_position, _ = self.person_tf()
            while self.last_position is None:
                self.last_position, _ = self.person_tf()
                
            self.speech.talk("I'm ready to follow you. Please start walking slowly.")
            """Loop principal do nó"""
            while not rospy.is_shutdown():
                self._start_orientation_pub.publish(self.msg)
                if self.id_detected:
                    position, _ = self.person_tf()
                    if position:
                        orientation = self.calculate_orientation(self.last_position, position)
                        rospy.sleep(3)
                        if self.check_for_movement(position):
                            rospy.loginfo("Sending goal to move_base")
                            self.send_goal_to_move_base(position, orientation, wait_for_result=False)
                            # Atualiza a última posição após enviar o objetivo
                            self.last_position = position
                            self.last_goal_pos = position
                            self.last_goal_rot = orientation
                            counter_for_stop = 0
                        else:
                            counter_for_stop += 1
                            if counter_for_stop >= 5:
                                rospy.loginfo("No significant movement detected for 5 iterations. (15 seconds)")
                                self.speech.talk("Do you want me to stop following you? Answer after the bip.")
                                answer = self.speech.search(self.possible_answers) 
                                if answer == 'yes':
                                    self.speech.talk("Ok, I'll stop following you.")
                                    break
                                else:
                                    counter_for_stop = 0 	

                else:
                    if self.person_detected:
                        rospy.loginfo("Person detected, but ID not found.")
                        rospy.sleep(1)
                    else:
                        rospy.loginfo("No person detected.")
                        rospy.sleep(1)
            self.msg = False
            self._start_orientation_pub.publish(self.msg)
            return True

    def follow_handler(self, req):
        return NavigationAndFollow.main()

if __name__ == '__main__':
    rospy.init_node('Navigation_And_Follow_Task')
    NavigationAndFollow = NavigationAndFollow()
    rospy.spin()
    rospy.loginfo('Navigation and Follow task finished')
    exit(0)


### Dedico esse código aos meus pais, meus irmãos (de sangue e de coração) e aos meus mentores (Plínio e Khaled)
### Creditos de ajuda intelectual ao capitão da equipe
### As long as the code is public you can use it with no problems and no obligations, obviously if it suits your use case, but if you can, leave a credit :)
