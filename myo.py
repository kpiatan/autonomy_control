#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Float32, Int16, Float32MultiArray
from geometry_msgs.msg import Vector3
from ros_myo.msg import EmgArray
from numpy import mean, sqrt, square, arange
from scipy.fftpack import fft, ifft

import tf
import numpy as np

window_size = 50 # janela do valor rms das leituras dos 8 sensores emg
ang_size = 50 # janela para calcular a media das leituras de roll, pitch e yaw

emg_data = np.zeros((window_size,8))
rms = np.zeros((8))
std = np.zeros((8))
var = np.zeros((8))
angular_data = np.zeros((ang_size,3))
d_roll = 0 #variacao do roll dentro da janela ang_size
d_pitch = 0
d_yaw = 0

def emgCallback(data):
    global emg_data
    emg_data[0] = data.data
    emg_data = np.roll(emg_data, 1, axis=0)

    return

def poseCallback(data):
    global angular_data
    angular_data[0][0] = data.x
    angular_data[0][1] = data.y
    angular_data[0][2] = data.z
    angular_data = np.roll(angular_data, 1, axis=0)

    return

def myo():

    rospy.Subscriber('/myo_raw/myo_emg', EmgArray, emgCallback)
    rospy.Subscriber('/myo_raw/myo_ori', Vector3, poseCallback)
    pub_rms = rospy.Publisher('/myo/rms', Float32, queue_size=10)
    pub_dang = rospy.Publisher('/myo/delta_ang', Vector3, queue_size=10)
    rospy.init_node('read_myo_node', anonymous=True)
    rate = rospy.Rate(50) # hz

    while not rospy.is_shutdown():
        global emg_data
        global roll, pitch, yaw
        global rms, std, var
        global angular_data

        emg_data_t = np.transpose(emg_data)
        for i in range(0,8):
            rms[i] = sqrt(mean(square(emg_data_t[i])))
            #std[i] = np.std(emg_data_t[i])
            #var[i] = np.var(emg_data_t[i])

        pub_rms.publish(rms.sum()/rms.size)
        media_angdata = np.sum(angular_data,axis=0)/ang_size
        diferenca = np.subtract(angular_data[0],media_angdata)
        d_yaw = diferenca[0]
        d_pitch = diferenca[1]
        d_roll = diferenca[2]
        print  d_roll, d_pitch, d_yaw
        pub_dang.publish(d_roll, d_pitch, d_yaw)
        rate.sleep()

if __name__ == '__main__':
    try:
        myo()
    except rospy.ROSInterruptException:
        pass