import numpy as np
import matplotlib.pyplot as plt
import random
import math
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import networkx as nx
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, floor
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from shapely.geometry import Point
from shapely.geometry import LineString
from shapely.geometry import Polygon
from time import sleep

from descartes.patch import PolygonPatch


class RRT:
    def __init__(self, start, goal, map_dims, obstacles, bias, epsilon, max_iter):
        self.goal = goal
        self.start = start
        self.width = map_dims[0]
        self.height = map_dims[1]
        self.obstacles = obstacles
        self.bias = bias
        self.max_iter = max_iter
        self.epsilon = epsilon

        self.show_map(obs_set, map_dims)
        self.G = nx.Graph()
        self.G.add_node((start[0], start[1]))
        
        
    #Desenha o mapa com obstáculos
    def show_map(self, obs_set, map_dims):
        fig = plt.figure(figsize=(8,5), dpi=100)
        ax = fig.add_subplot(111, aspect='equal') 

        for obs in obs_set:
            ax.add_patch(PolygonPatch(obs, facecolor='gray'))
        
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        plt.show()
    
    #Amostra um ponto livre no mapa
    def sample_point(self):            
        x = random.uniform(1, self.width-1) #limitando pelo tamanho da parede
        y = random.uniform(1, self.height-1)
        p = Point([x,y])
        
        #se o ponto intercepta um obstáculo, retorna falso
        for o in self.obstacles:
            if p.intersects(o):
                return False, p
        return True, p
    
    #distância euclidiana
    def distance(self, p1, p2):
        px = (p1[0] - p2.x)**2
        py = (p1[1] - p2.y)**2
        return (px + py)**(0.5)
    
    #encontra o nó vizinho mais próximo à amostra fornecida
    def nearest_neighbor(self, sample):
        d = 1000
        n = self.G[start]
        for node in self.G:
            dist = self.distance(node, sample)
            if dist < d:
                d = dist
                n = node
        return d, n
    
    #expande o nó mais próximo em direção à amostra numa distancia fixa epsilon
    def expand_tree(self, n, d, sample):
        theta = math.asin((sample.y-n[1])/d)
        #x = n[0]+self.epsilon*np.cos(theta)
        #y = n[1]+self.epsilon*np.sin(theta)
        x = sample.x
        y = sample.y
        if x < 0:
            x = 0
        if x > self.width:
            x = self.width
        if y < 0:
            y = 0
        if y > self.height:
            y = self.height
        
        p = Point([x,y])
        node = Point([n[0], n[1]])
        line = LineString([p,n])
        for o in self.obstacles:
            if line.intersects(o):
                return x, y, False
        return x, y, True
    
        #for s in samples:
        #    print(s)
        #    plt.plot(*s[0].xy,'b*')
        
    #tenta fazer uma conexão direta do nó ao alvo
    def try_connect_goal(self, x, y):
        p = Point([x,y])
        g = Point([goal[0],goal[1]])
        line = LineString([p,g])
        for o in self.obstacles:
            if line.intersects(o):
                return False
        self.G.add_node((goal[0], goal[1]))
        self.G.add_edge((x,y),(goal[0], goal[1]))
        return True
    
    #executa rrt
    def run(self):
        i = 0
        path_found = False
        path=[]
        while i < self.max_iter and not path_found:
            is_sample_valid = False
            sample = (0,0)
            while not is_sample_valid:
                is_sample_valid, sample = self.sample_point();            
            sample = random.choices(population=[Point([goal[0],goal[1]]),sample],weights=[bias,1-bias])
            d, n = self.nearest_neighbor(sample[0])
            x, y, is_valid_connection = self.expand_tree(n, d, sample[0])
            if not is_valid_connection:
                continue
            self.G.add_node((x, y))
            self.G.add_edge(n, (x,y))
            
            path_found = self.try_connect_goal(x,y)
            if(path_found):
                print("Path has been found after ",i," iterations!")
                path = nx.shortest_path(self.G, source=(self.start[0], self.start[1]), target=(self.goal[0], self.goal[1]))
                
                pos = {node:(node[0], node[1]) for node in self.G.nodes()}
                pos_path = {node:(node[0], node[1]) for node in path}
                f = plt.figure(figsize=(8,5), dpi=100)
                ax = f.add_subplot(111, aspect='equal') 

                for obs in obs_set:
                    ax.add_patch(PolygonPatch(obs, facecolor='gray'))
                
                ax.set_xlim(0, self.width)
                ax.set_ylim(0, self.height)
                #nx.draw_networkx_nodes(self.G, pos, nodelist=path, node_size=100, node_color='b')
                path_edges = list(zip(path,path[1:]))
                nx.draw(self.G, pos, ax, font_size=3, with_labels=False, node_size=20, node_color='b', width=1, edge_color='gray')
                nx.draw_networkx_nodes(self.G,pos,nodelist=path, node_size=20, node_color='r')
                nx.draw_networkx_edges(self.G,pos,edgelist=path_edges,edge_color='r',width=2)
                plt.show()
                return path_found, path
            i=i+1
        print("Could not find path after ",i, " iterations.")
        pos = {node:(node[0], node[1]) for node in self.G.nodes()}
        #nx.draw_networkx_nodes(self.G, pos, nodelist=path, node_size=100, node_color='b')
       
        nx.draw(self.G, pos, ax, font_size=3, with_labels=False, node_size=20, node_color='b', width=1, edge_color='gray')
        
        plt.show()
        return path_found, path


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


if __name__ == '__main__':
    try:
        freq = 100
        obstacle1 = Polygon([(1.5,20),(1.5, 17.5),(6.5,17.5),(6.5,20)])
        obstacle2 = Polygon([(8,18.5),(8,14.5),(12.5,14.5), (12.5,18.5)])
        obstacle3 = Polygon([(20, 8.2),(14.75, 8.5),(14,8),(12,9),(8.5,9),(8.5,12.5),(13,12.5),(14.35,14.5),(17.2,13.5),(17.3,12.5),(20,12.5)])
        obstacle4 = Polygon([(3.5,12.3),(3.5,16),(6.75,16),(6.75,12.5)])
        obstacle5 = Polygon([(2.5,5),(2.5,9.3),(5.5,9.3),(5.5,5)])
        obstacle6 = Polygon([(8,4),(8,7.5),(13.5,7.5),(13.5,4.5)])
        obstacle7 = Polygon([(15,0),(15,4),(20,4),(20,0)])
        obs_set = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7]


        rospy.init_node("RRT_node") #inicializa o no "este no"
        rate = rospy.Rate(freq)
        pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
        rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
        sleep(1)

        x_goal, y_goal = input('(x_goal, y_goal)').split()
        x_goal, y_goal = [float(i) for i in [x_goal, y_goal]]

        start = (x_n,y_n)
        goal = (2,2)
        map_dims = np.array([20, 20]) 
        bias = 0.05
        max_dist = 1.0
        max_iter = 1000
        rrt = RRT(start, goal, map_dims, obs_set, bias, max_dist, max_iter)
        #rrt.show_map(obs_set, map_dims)
        found, path = rrt.run()
        K = 0.5
        vel = Twist()
        if(found):
            for node in path:
                qgoal = np.array([node[0], node[1]])
                cell_reached = False
                while not cell_reached:
                    q = np.array([x_n, y_n])
                    error = qgoal - q
                    if (np.linalg.norm(error) < 0.15):
                        cell_reached = True
                    v_x = K*error[0]
                    v_y = K*error[1]
                    vel.linear.x = v_x
                    vel.linear.y = v_y
                    pub_stage.publish(vel)
                    rate.sleep()

        print('pose: ', start)

        planConverted = []
        for node in path:
            print(node)
    except rospy.ROSInterruptException:
        pass
