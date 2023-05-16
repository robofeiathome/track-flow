import time
from collections import deque
# Soon i will add all the functions here to make it easier to work
# on develop, currentrly not working at all, missing various parts of the code
waypoints = deque()
robot = None  # Initialize your robot object here

def get_person_position():#chamar o flow_tracker p pegar as posicoes(x,y) e o flow 
    pass

def distance_to_point(robot_position, point):#Criar a funcao que calcula a distancia entre o robo e o bbox(z) com point cloud
    pass

def move_to_point(point):#criar a funcao que faz com que o robo va para o local desejado, ex. goto
    pass

while True:
    # Get the person's position and add it to the waypoints deque every 5 seconds
    position = get_person_position()
    waypoints.append(position)
    time.sleep(5)

    if len(waypoints) > 0:
        # Get the first waypoint from the list
        waypoint = waypoints[0]

        # Check if the robot is close enough to the waypoint
        if distance_to_point(robot.get_position(), waypoint) < threshold: #definir um treshhold e testar para refinar
            # If the robot is close enough, remove the first waypoint from the list
            waypoints.popleft()
        else:
            # If the robot is not close enough, move the robot to the waypoint
            move_to_point(waypoint)
