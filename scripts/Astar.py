#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, floor
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

global x_n, y_n, theta_n

class Node():
    def __init__(self, value, x, y):
        self.value = value
        self.gCost = 0
        self.hCost = 0
        self.x = x
        self.y = y
        self.parent = None
        self.path = None

    def fCost(self):
        return self.gCost + self.hCost

class Astar():
    def __init__(self, map, goal, start, cell_size):
        self.map = map
        self.goal = goal
        self.start = start
        self.cell_size = cell_size
        self.goal2d = self.goal_to_node2d()
        self.rows = len(self.map)
        self.cols = len(self.map[0])
        self.targetNode = self.map[self.goal2d[0]][self.goal2d[1]]
        #self.map[self.goal2d[0]][self.goal2d[1]] = 2
        #self.map[self.goal2d[1]][self.goal2d[0]].value = 2
        #print(*self.map)
        

    def goal_to_node2d(self):
        #goal: x,y
        goal2d = np.array([0,0])
        goal2d[0] = self.goal[0]/self.cell_size
        goal2d[1] = -self.goal[1]/self.cell_size
        return goal2d

    def node2d_to_goal(self, cell):
        x = cell.x*self.cell_size + cell_size/2
        y = -cell.y*self.cell_size - cell_size/2
        return (x,y)

    def isGoalValid(self):
        if(self.map[self.goal2d[0]][self.goal2d[1]].value == 1):
            return False
        elif(self.map[self.goal2d[0]][self.goal2d[1]].value == 0):
            return True

    def getNeighbors(self, node):
        neighbors = []

        for x in range(-1,2):
            for y in range(-1,2):
                if(x == 0 and y == 0):
                    continue
                checkX = node.x + x
                checkY = node.y + y
                #print('check:',x,y)
                if(checkX >= 0 and checkX < self.rows and checkY >= 0 and checkY < self.cols):
                    neighbors.append(self.map[checkX][checkY])

        return neighbors

    def getDistance(self, nodeA, nodeB):
        distX = abs(nodeA.x - nodeB.x)
        distY = abs(nodeA.y - nodeB.y)

        if(distX > distY):
            return 14*distY + 10*(distX - distY)
        else:
            return 14*distX + 10*(distY - distX)

    def tracePath(self, startNode, endNode):
        path = []
        currentNode = endNode
        while(currentNode is not startNode):
            path.append(currentNode)
            currentNode = currentNode.parent
            #print('node:', currentNode)
        path.reverse()
        #print('path:',path)
        return path

    def findPath(self):
        openSet = []
        closeSet = []
        print(vars(self.map[self.start[0]][self.start[1]]))
        startNode = self.map[self.start[0]][self.start[1]]
        openSet.append(startNode)
        while(len(openSet) > 0):
            currentNode = openSet[0]
            for i in range(1,len(openSet)):
                #print(openSet[i].fCost())
                if(openSet[i].fCost() < currentNode.fCost() or (openSet[i].fCost() == currentNode.fCost() and openSet[i].hCost < currentNode.hCost)):
                    currentNode = openSet[i]
            
            #print('in while: ', currentNode.x, currentNode.y, currentNode.fCost())
            #print('goal: ', self.goal2d[0] , self.goal2d[1])
            openSet.remove(currentNode)
            closeSet.append(currentNode)

            if(currentNode.x == self.goal2d[0] and currentNode.y == self.goal2d[1]):
                print('search done')
                self.path = self.tracePath(startNode, self.targetNode)
                return
            neighbors = self.getNeighbors(currentNode)

            for neighbor in neighbors:
                #print(vars(neighbor))
                if(neighbor.value == 1 or (neighbor in closeSet)):
                    print('continue')
                    continue

                newMovementCostToNeighbor = currentNode.gCost + self.getDistance(currentNode, neighbor)
                if(newMovementCostToNeighbor < neighbor.gCost or not (neighbor in openSet)):
                    neighbor.gCost = newMovementCostToNeighbor
                    neighbor.hCost = self.getDistance(neighbor, self.targetNode)
                    neighbor.parent = currentNode
                    #print(neighbor.gCost)
                    if(neighbor not in openSet):
                        openSet.append(neighbor)
            print('next')

    def plotGrid(self):
        for i in range(len(self.map)):
            line = []
            linefCost = []
            for j in range(len(self.map[0])):
                line.append(self.map[i][j].value)
                linefCost.append(self.map[i][j].gCost)
            #print(line)
            print(linefCost)

def refference_trajectory(x_goal, y_goal):
    x_ref = x_goal
    y_ref = y_goal
    Vx_ref = 0
    Vy_ref = 0
    return (x_ref, y_ref, Vx_ref, Vy_ref)

# Rotina para a geracao da entrada de controle
def trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref, Kp, Usat):
    global x_n, y_n, theta_n
    Ux = Vx_ref + Kp * (x_ref - x_n)
    Uy = Vy_ref + Kp * (y_ref - y_n)
    absU = sqrt(Ux ** 2 + Uy ** 2)
    if (absU > Usat):
        Ux = Usat * Ux / absU
        Uy = Usat * Uy / absU
    return (Ux, Uy)

# Rotina feedback linearization
def feedback_linearization(Ux, Uy, d):
    global x_n, y_n, theta_n
    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy
    return (VX, WZ)
            

# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n
    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo
    return    

def calcDistance(x_n, y_n, x_d, y_d):
    return sqrt(((x_d - x_n)**2 + (y_d - y_n)**2))


def readImage(cell_size):
    fig = plt.figure(figsize=(8,8), dpi=100)
    img = 1 - mpimg.imread('../worlds/map_1.png')

    # Apenas para garantir que só teremos esses dois valores
    threshold = 0.5
    img[img > threshold] = 1
    img[img<= threshold] = 0
    map_dims = np.array([60, 60]) # Cave 

    # Escala Pixel/Metro
    print(img.shape)
    sy, sx = img.shape[0:2] / map_dims

    # Tamanho da célula do nosso Grid (em metros)

    rows, cols = (map_dims / cell_size).astype(int)
    #grid = np.zeros((rows, cols))
    grid = [[Node(0,0,0) for x in range(cols)] for y in range(rows)]
    # Preenchendo o Grid
    for r in range(rows):
        for c in range(cols):
            
            xi = int(c*cell_size*sx)
            xf = int(xi + cell_size*sx)
            
            yi = int(r*cell_size*sy)
            yf = int(yi + cell_size*sy)
            value = np.sum(img[yi:yf,xi:xf])
            if(value > threshold):
                value = 1
            else:
                value = 0
            
            node = Node(value, r, c)
            grid[r][c] = node
            
    return grid


def control(poses):
    #Tempo de simulacao no stage
    global x_n, y_n
    freq = 100
    Usat = 5
    d = 0.8
    Kp = 1

    #Define uma variavel que controlar a a frequencia de execucao deste no
    rate = rospy.Rate(freq)
    vel = Twist()
    sleep(0.2)

    # O programa do no consiste no codigo dentro deste while
    for pose in poses:
        print(pose)
        x_goal = pose[0]
        y_goal = pose[1]
        # Incrementa o tempo
        dist = calcDistance(x_n,y_n,x_goal,y_goal)
        while(dist > 0.5):
            [x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(x_goal, y_goal)

            [Ux, Uy] = trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref, Kp, Usat)

            [V_forward, w_z] = feedback_linearization(Ux, Uy, d)

            vel.linear.x = V_forward
            vel.angular.z = w_z
            pub_stage.publish(vel)
            dist = calcDistance(x_n, y_n, x_goal, y_goal)

            #Espera por um tempo de forma a manter a frequencia desejada
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("Astar_node") #inicializa o no "este no"
        pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
        rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
        
        cell_size = 2
        x_goal, y_goal = input('(x_goal, y_goal)').split()
        x_goal, y_goal = [float(i) for i in [x_goal, y_goal]]
        grid = readImage(cell_size)
        start_x = floor(x_n/cell_size)
        start_y = floor(-y_n/cell_size)
        print('pose: ', start_x, start_y)

        Astar = Astar(grid, np.array([x_goal, y_goal]), np.array([start_x,start_y]), cell_size)
        if(not Astar.isGoalValid()):
            print('Posicao de alvo invalida')
            exit()

        Astar.findPath()
        path = Astar.path
        planConverted = []
        for node in path:
            pose = Astar.node2d_to_goal(node)
            planConverted.append(pose)
            print(pose)
        Astar.plotGrid()
        control(planConverted)
    except rospy.ROSInterruptException:
        pass
