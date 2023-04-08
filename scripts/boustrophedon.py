#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, floor
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from time import sleep
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

global x_n, y_n, theta_n

class Node():
    def __init__(self, value, x, y):
        self.value = value
        self.x = x
        self.y = y


class bcd():
    def __init__(self, map, start, cell_size, cells):
        self.map = map
        self.start = start
        self.cell_size = cell_size
        self.rows = len(self.map)
        self.cols = len(self.map[0])
        self.cells = cells

    def plotGrid(self):
        for i in range(len(self.map)):
            line = []
            for j in range(len(self.map[0])):
                line.append(self.map[i][j].value)
            print(line)

    def node2d_to_goal(self, cell):
        x = cell.x*self.cell_size + cell_size/2
        y = -cell.y*self.cell_size - cell_size/2
        return (x,y)

    def coveragePath(self):
        valuecell = 2
        cellPaths = []
        for cell in self.cells:
            path = []
            for i in range(self.cols):
                if (i % 2 == 0):
                    #baixo pra cima
                    for j in range(self.rows-1,0,-1):
                        point = Point(i,-j)
                        #print(point)
                        if (self.map[j][i].value == 1):
                            continue
                        if (cell.intersects(point)):
                            print('contains')
                            self.map[j][i].value = valuecell
                            pose = Node(0,i,j)
                            path.append(self.node2d_to_goal(pose))
                else:
                    #cima pra baixo
                    for j in range(self.rows):
                        point = Point(i,-j)
                        #print(point)
                        if (self.map[j][i].value == 1):
                            continue
                        if (cell.intersects(point)):
                            print('contains')
                            self.map[j][i].value = valuecell
                            pose = Node(0,i,j)
                            path.append(self.node2d_to_goal(pose))
            cellPaths.append(path)
            valuecell = valuecell + 1
        return cellPaths

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
     
def calcDistance(x_n, y_n, x_d, y_d):
    return sqrt(((x_d - x_n)**2 + (y_d - y_n)**2))


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


def readImage(cell_size):
    fig = plt.figure(figsize=(8,8), dpi=100)
    img = 1 - mpimg.imread('../worlds/map_3.png')

    # Apenas para garantir que só teremos esses dois valores
    threshold = 0.5
    img[img > threshold] = 1
    img[img<= threshold] = 0
    map_dims = np.array([60, 60]) 

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

if __name__ == '__main__':
    try:
        rospy.init_node("BCD_node") #inicializa o no Boustrophedon
        pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
        rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
        sleep(1)
        cell_size = 2
        grid = readImage(cell_size)
        start_x = floor(x_n/cell_size)
        start_y = floor(-y_n/cell_size)
        print('pose: ', start_x, start_y)

        cell1 = Polygon([(1,-1),(2,-1),(2,-28),(1,-28)])
        cell2 = Polygon([(3,-1),(11,-1),(11,-6),(3,-6)])
        cell3 = Polygon([(3,-12),(11,-12),(11,-28),(3,-28)])
        cell4 = Polygon([(12,-1),(15,-1),(15,-28),(12,-28)])
        cell5 = Polygon([(16,-1),(20,-1),(20,-14),(16,-20)])
        cell6 = Polygon([(21,-1),(24,-1),(24,-20),(21,-14)])
        cell7 = Polygon([(25,-1),(28,-1),(28,-28),(25,-28)])
        cell8 = Polygon([(16,-23),(24,-23),(24,-28),(16,-28)])
        cells = [cell1,cell2,cell4,cell3,cell5,cell6,cell7,cell8]

        img = plt.imread("../worlds/map_3.png")
        fig, ax = plt.subplots()
        ax.imshow(np.flipud(img), extent=[0,29,0,-29])
        for cell in cells:
            x,y = cell.exterior.xy
            ax.plot(x,y)
        plt.gca().invert_yaxis()
        plt.show()
        bcd = bcd(grid, np.array([start_x,start_y]), cell_size, cells)
        cellPaths = bcd.coveragePath()
        cellPaths[len(cellPaths)-1].reverse()
        for path in cellPaths:
            print("Moving to next cell")
            control(path)
        bcd.plotGrid()
        #control(planConverted)
    except rospy.ROSInterruptException:
        pass
