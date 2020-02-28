#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist, Quaternion,TwistStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,Float32, Int16, UInt8
from sensor_msgs.msg import Joy
import tf
from numpy import mean, sqrt, square, arange

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
d_roll = 0 #variacao dentro de uma janela de medicoes dos angulos da myo
d_pitch = 0
d_yaw = 0

joyX = 0 #vel linear lida do topico cmd_vel publicado pelo joystick
joyZ = 0 #vel angular lida do topico cmd_vel publicado pelo joystick
autX = 0 #vel linear lida do topico cmd_vel publicado pelo simulacao.py
autZ = 0 #vel linear lida do topico cmd_vel publicado pelo simulacao.py

iniciar_dados = 0 # rotina para mostrar dados na tela
vetor_dados = [] # vetor que acumula os erros do cordao de solda

autonomy_level = 1
autonomy_level_int = 1
autonomy_level_int_ant = 1

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

def dadosCallback(data):
    global iniciar_dados
    iniciar_dados = data.data

    return

def rmsCallback(data):
    global rms
    rms = data.data

    return

def dangCallback(data):
    global d_roll, d_pitch, d_yaw
    d_roll = data.x
    d_pitch = data.y
    d_yaw = data.z

    return

def gestCallback(data):

    return
    

def talker():
    global d
    global yaw
    global gravidade_x
    global gravidade_y
    global gravidade_z
    global rms, vel_linear, theta
    global joyX, joyZ, autX, autZ
    global autonomy_level, autonomy_level_int, autonomy_level_int_ant
    global d_roll, d_pitch, d_yaw
    global iniciar_dados
    global vetor_dados

    fuzzy_autonomy.inicializaFuzzy()
    

    pubAutonomy = rospy.Publisher('autonomy_level', Float32, queue_size=10)
    pubVel = rospy.Publisher('air1/cmd_vel',Twist,queue_size=10)
    pubHap = rospy.Publisher('myo_raw/vibrate',UInt8,queue_size=10)
    rospy.Subscriber('joy/cmd_vel', Twist, joyvelCallback)
    rospy.Subscriber('sim/cmd_vel', Twist, simvelCallback)
    rospy.Subscriber('air1/lrs36', LaserScan, laserCallback)
    rospy.Subscriber('air1/odon', Odometry, odonCallback)
    rospy.Subscriber('air1/twist', TwistStamped, twistCallback)
    rospy.Subscriber('joy', Joy, joyCallback)
    rospy.Subscriber('joy/iniciar_dados', Int16, dadosCallback)
    rospy.Subscriber('/myo/rms', Float32, rmsCallback)
    rospy.Subscriber('/myo/delta_ang', Vector3, dangCallback)
    rospy.Subscriber('/myo_raw/myo_gest', Vector3, gestCallback)    
    rospy.init_node('select_autonomy_node', anonymous=True)
    rate = rospy.Rate(100) # hz

    while not rospy.is_shutdown():
        data = d
        if data != 0:
            erro_x = 0
            scan = ajusteTanque.laserScanToPointCloud(data)
            tanque = ajusteTanque.calcularPosicaoTanque(scan)

            if tanque != -1:
                sinal = ajusteTanque.determinarSinal(scan,tanque)
                erro_x = ajusteTanque.calcularCentroSolda(sinal)
                erro_orientacao =  0

                if iniciar_dados == 1:
                    vetor_dados.append(erro_x)
                if iniciar_dados == 0:
                    rms_vetor = sqrt(mean(square(vetor_dados)))
                    print "RMS Dados:", rms_vetor

                autonomy_level=fuzzy_autonomy.calculateAutonomy(rms,theta,erro_x, d_roll)
                #autonomy_level = 1 #teste
                pubAutonomy.publish(autonomy_level)


        msg_cmd_vel = Twist()

        if joyX != 0:
            joyX = joyX - d_pitch*0.1 # um peso dos angulos do controle no valor da velocidade, pitch para cima positivo
        if joyZ != 0:
            joyZ = joyZ + d_roll*0.1  # roll sentido horario positivo

        if autonomy_level <= 1: # modo manual
            msg_cmd_vel.linear.x = joyX # velocidade totalmente pelo controle
            msg_cmd_vel.angular.z = joyZ
        elif autonomy_level > 1 and autonomy_level <= 2: # modo compartilhado
            msg_cmd_vel.linear.x = (autonomy_level-1)*autX + (2-autonomy_level)*joyX # uma parte da velocidade eh do controle e outra do modo autonomo
            msg_cmd_vel.angular.z = (autonomy_level-1)*autZ + (2-autonomy_level)*joyZ
        elif autonomy_level > 2 and autonomy_level <= 3: # modo supervisorio
            msg_cmd_vel.linear.x = joyX
            msg_cmd_vel.angular.z = autZ
        elif autonomy_level > 3: # modo autonomo
            msg_cmd_vel.linear.x = autX
            msg_cmd_vel.angular.z = autZ

        #limitadores de velocidade
        if msg_cmd_vel.linear.x > 1.0:
            msg_cmd_vel.linear.x = 1.0
        if msg_cmd_vel.angular.z > 2.5:
            msg_cmd_vel.angular.z = 2.5

        pubVel.publish(msg_cmd_vel)

        autonomy_level_int_ant = autonomy_level_int

        # vibracao de acordo com o nivel de autonomia
        if autonomy_level <= 1:
            autonomy_level_int = 1
        elif autonomy_level > 1 and autonomy_level <= 2:
            autonomy_level_int = 2
        elif autonomy_level > 2 and autonomy_level <= 3:
            autonomy_level_int = 3
        elif autonomy_level > 3:
            autonomy_level_int = 4

        if autonomy_level_int > autonomy_level_int_ant:
            pubHap.publish(2)
        elif autonomy_level_int < autonomy_level_int_ant:
            pubHap.publish(1)
        else:
            pubHap.publish(0)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass