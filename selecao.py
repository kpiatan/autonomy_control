#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist, Quaternion,TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,Float32, Int16, UInt8
from sensor_msgs.msg import Joy
import tf

import random
import time
import math

import filtro
import ajusteTanque
import fuzzy_autonomy
import time

import os

d = 0
yaw = -10
gravidade_x = 0
gravidade_y = 0
gravidade_z = 1

vel_linear = 0 # vel_linear lida do joystick
theta = 0 # vel_angular lida do joystick
rms = 0 # valor rms da myo

joyX = 0 #vel linear lida do topico cmd_vel publicado pelo joystick
joyZ = 0 #vel angular lida do topico cmd_vel publicado pelo joystick
autX = 0 #vel linear lida do topico cmd_vel publicado pelo simulacao.py
autZ = 0 #vel linear lida do topico cmd_vel publicado pelo simulacao.py

autonomy_level = 1

def laserCallback(data):
    global d
    d = data

    return

def odonCallback(data):
    global yaw
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    explicit_quat = [x,y,z,w]

    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(explicit_quat)
    # print yaw
    return

def twistCallback(data):
    global gravidade_x
    global gravidade_y
    global gravidade_z
    if data != 0:
        gravidade_x = data.twist.linear.x/(math.fabs(data.twist.linear.x) + math.fabs(data.twist.linear.y) + math.fabs(data.twist.linear.z))
        gravidade_y = data.twist.linear.y/(math.fabs(data.twist.linear.x) + math.fabs(data.twist.linear.y) + math.fabs(data.twist.linear.z))
        gravidade_z = data.twist.linear.z/(math.fabs(data.twist.linear.x) + math.fabs(data.twist.linear.y) + math.fabs(data.twist.linear.z))
    #print gravidade_x,gravidade_y,gravidade_z
    return

def joyvelCallback(data):
    global joyX, joyZ
    joyX = data.linear.x
    joyZ = data.angular.z

    return

def simvelCallback(data):
    global autX, autZ
    autX = data.linear.x
    autZ = data.angular.z

    return
    
def joyCallback(data):
    global vel_linear, theta
    vel_linear = data.axes[1]
    theta = data.axes[0]

    return

def rmsCallback(data):
    global rms
    rms = data.data

    return
    

def talker():
    global d
    global yaw
    global gravidade_x
    global gravidade_y
    global gravidade_z
    global rms, vel_linear, theta
    global joyX, joyZ, autX, autZ
    global autonomy_level

    fuzzy_autonomy.inicializaFuzzy()

    pubAutonomy = rospy.Publisher('autonomy_level', Int16, queue_size=10)
    pubVel = rospy.Publisher('air1/cmd_vel',Twist,queue_size=10)
    pubHap = rospy.Publisher('myo_raw/vibrate',UInt8,queue_size=10)
    rospy.Subscriber('joy/cmd_vel', Twist, joyvelCallback)
    rospy.Subscriber('sim/cmd_vel', Twist, simvelCallback)
    rospy.Subscriber('air1/lrs36', LaserScan, laserCallback)
    rospy.Subscriber('air1/odon', Odometry, odonCallback)
    rospy.Subscriber('air1/twist', TwistStamped, twistCallback)
    rospy.Subscriber('joy', Joy, joyCallback)
    rospy.Subscriber('/myo/rms', Float32, rmsCallback)
    rospy.init_node('select_autonomy_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        data = d
        if data != 0:
            erro_x = 0
            scan = ajusteTanque.laserScanToPointCloud(data)
            tanque = ajusteTanque.calcularPosicaoTanque(scan)

            if tanque != -1:
                sinal = ajusteTanque.determinarSinal(scan,tanque)
                erro_x = ajusteTanque.calcularCentroSolda(sinal)
                #pubErro.publish(erro_x)
                erro_orientacao =  0
                print erro_x
                #erroacumulado = erroacumulado + erro_x*erro_x
                #print("ErroAcumulado:")
                #print erroacumulado
                #elapsed_time = time.time() - start_time
                #print("Tempo:")
                #print elapsed_time
                #erro_por_tempo = math.sqrt(erroacumulado)/elapsed_time
                #print("metrica:")
                #print erro_por_tempo

                autonomy_level=fuzzy_autonomy.calculateAutonomy(rms,float(theta),erro_x)
                #autonomy_level=fuzzy_autonomy.calculateAutonomy(30,erro_x,0.000)
                pubAutonomy.publish(autonomy_level)


        msg_cmd_vel = Twist()
        if autonomy_level <= 1:
            msg_cmd_vel.linear.x = joyX
            msg_cmd_vel.angular.z = joyZ
        elif autonomy_level > 1 and autonomy_level <= 2:
            msg_cmd_vel.linear.x = (autonomy_level-1)*autX + (2-autonomy_level)*joyX
            msg_cmd_vel.angular.z = (autonomy_level-1)*autZ + (2-autonomy_level)*joyZ
        elif autonomy_level > 2:
            msg_cmd_vel.linear.x = autX
            msg_cmd_vel.angular.z = autZ
                
        pubVel.publish(msg_cmd_vel)

        #pubHap.publish(1)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass