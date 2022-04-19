#!/user/bin/python3

import rospy
import time
import os
import ADS1263
import RPi.GPIO as GPIO
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension

# Modify according to actual voltage
# external AVDD and AVSS(Default), or internal 2.5V
REF = 5.08

class mykalman():

    def __init__(self):

        self.sz = 1 # size of array
        self.Q = 1e-6 # process variance
        # allocate space for arrays
        self.xhat=np.zeros(1)      # a posteri estimate of x
        self.P=np.zeros(1)         # a posteri error estimate
        self.xhatminus=np.zeros(1) # a priori estimate of x
        self.Pminus=np.zeros(1)    # a priori error estimate
        self.K=np.zeros(1)         # gain or blending factor

        self.R = 0.01**2 # estimate of measurement variance, change to see effect

        # intial guesses
        self.xhat = 0.0
        self.P = 1.0
        self.A = 1
        self.H = 1

    def update(self,z):
         # time update
        self.xhatminus = self.A * self.xhat  #X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),A=1,BU(k) = 0
        self.Pminus = self.A * self.P+self.Q      #P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=1

        # measurement update
        self.K = self.Pminus/( self.Pminus+self.R ) #Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1
        self.xhat = self.xhatminus+self.K*(z-self.H * self.xhatminus) #X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)], H=1
        self.P = (1-self.K * self.H) * self.Pminus #P(k|k) = (1 - Kg(k)H)P(k|k-1), H=1
        return self.xhat


class LoadCellProcess:
    def __init__(self):
        self.ADC = ADS1263.ADS1263()
        rospy.loginfo("Setting up the node")
        rospy.init_node("loadcell_ros_interface", anonymous=True)
        self.force_pub = rospy.Publish('/loadcell_forces', Float32MultiArray, queue_size=1)
        self.fx=np.zeros((1, 100))
        self.fy=np.zeros((1, 100))
        self.fz=np.zeros((1, 100))
        self.fx_filtered=np.zeros((1, 100))
        self.fy_filtered=np.zeros((1, 100))
        self.fz_filtered=np.zeros((1, 100))
        self.kfx=mykalman()
        self.kfy=mykalman()
        self.kfz=mykalman()
        self.fx_zero= 0 #2.4543
        self.fy_zero= 0 #2.444
        self.fz_zero= 0 #2.454

    def initialize(self):
        if(self.ADC.ADS1263_init_ADC1('ADS1263_7200SPS') == -1):
            return False
        else:
            self.ADC.ADS1263_SetMode(0)

            return True

    def acquire_data(self):
        ADC_Value = self.ADC.ADS1263_GetAll()    # get ADC1 value
        for i in range(0, 3):
            if(ADC_Value[i]>>31 ==1):
                if i==0:
                    raw=-1*(REF*2 - ADC_Value[i] * REF / 0x80000000)
                    self.fx=(raw-self.fx_zero)*200
                    self.fx_filtered=self.kfx.update(fx)
                if i==1:
                    raw=-1*(REF*2 - ADC_Value[i] * REF / 0x80000000)
                    self.fy=(raw-self.fy_zero)*200
                    self.fy_filtered=self.kfy.update(fy)
                if i==2:
                    raw=-1*(REF*2 - ADC_Value[i] * REF / 0x80000000)
                    self.fz=(raw-self.fz_zero)*200
                    self.fz_filtered=self.kfz.update(fz)
                #print("ADC1 IN%d = -%lf" %(i, (REF*2 - ADC_Value[i] * REF / 0x80000000)))
            else:
                #print("ADC1 IN%d = %lf" %(i, (ADC_Value[i] * REF / 0x7fffffff)))   # 32bit
                if i==0:
                    raw=float((ADC_Value[i] * REF / 0x7fffffff))
                    self.fx=(raw-self.fx_zero)*200
                    self.fx_filtered=self.kfx.update(fx)
                if i==1:
                    raw=float((ADC_Value[i] * REF / 0x7fffffff))
                    self.fy=(raw-self.fy_zero)*200
                    self.fy_filtered=self.kfy.update(fy)
                if i==2:
                    raw=float((ADC_Value[i] * REF / 0x7fffffff))
                    self.fz=(raw-self.fz_zero)*200
                    self.fz_filtered=self.kfz.update(fz)

    def publish_forces(self):
        self.acquire_data()
        forces = Float32MultiArray(data=[self.fx_filtered,self.fy_filtered,self.fz_filtered])
        self.force_pub.publish(forces)


    def run(self):
        if not self.initialize():
            exit()

        rate = rospy.Rate(200)
        try:
            while not rospy.is_shutdown():
                self.publish_forces()
                rate.sleep()
        except rospy.ROSInterruptException:
            self.ADS.ADS1263_Exit()
            pass

if __name__ == '__main__':
    process = LoadCellProcess()
    process.run()
