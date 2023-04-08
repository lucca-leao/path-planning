import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import tf
import sys
import numpy as np

# Estados do robo
global x_n, y_n, theta_n, laserPoints, laserInfo
x_n = 0.0  # posicao x atual do robo
y_n = 0.0  # posicao y atual do robo
theta_n = 0.0  # orientacao atual do robo


def getClosestPoints():
    global laserPoints
    for i in range(0, len(laserPoints)-2):
        prev = laserPoints[i-1]
        next = laserPoints[i+1]
        if(prev > laserPoints[i] and next > laserPoints[i]):
            minPoints.append(laserPoints[i])


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

def moveFromObstacle(pub_vel):
    global laserInfo, theta_n
    least = laserInfo[0]
    for laser in laserInfo:
        if(laser[0] < least[0]):
            least = laser
    sensorBackwards = False
    goal_theta = theta_n + pi
    vel = Twist()
    freq = 100
    rate = rospy.Rate(freq)
    K = 0.1
    while(not sensorBackwards):
        error = goal_theta - theta_n
        if(error < 0.01):
            sensorBackwards = True
        vel.angular.z = error*K
        pub_vel.publish(vel)
        rate.sleep()
        print(theta_n)
    return

def callback_laser(data):
    global x_n, y_n, theta_n, laserPoints, laserInfo
    laserPoints = []
    laserInfo = []
    for index in range(len(data.ranges)-1):
        dist = data.ranges[index]
        angle = index*data.angle_increment + data.angle_min
        x = x_n + dist*cos(theta_n + angle)
        y = y_n + dist*sin(theta_n + angle)
        laserPoints.append((x,y))
        laserInfo.append((dist,angle))
    #print(laserInfo)


if __name__ == '__main__':
    try:
        rospy.init_node("GVD_node") #inicializa o no "este no"
        pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
        rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
        rospy.Subscriber("/base_scan", LaserScan, callback_laser)
        sleep(1)
        moveFromObstacle(pub_stage)
        #Define uma variavel que controlar a a frequencia de execucao deste no
        freq = 100
        rate = rospy.Rate(freq)
        vel = Twist()
        sleep(0.2)

    except rospy.ROSInterruptException:
        pass
