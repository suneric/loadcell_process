#!/usr/bin/python3

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
SCALE = 100

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
        self.force1_pub = rospy.Publisher('/loadcell1_forces', Float32MultiArray, queue_size=1)
        self.force2_pub = rospy.Publisher('/loadcell2_forces', Float32MultiArray, queue_size=1)

        self.kfx1=mykalman()
        self.kfy1=mykalman()
        self.kfz1=mykalman()
        self.fx1=np.zeros((1, 100))
        self.fy1=np.zeros((1, 100))
        self.fz1=np.zeros((1, 100))
        self.fx1_filtered=np.zeros((1, 100))
        self.fy1_filtered=np.zeros((1, 100))
        self.fz1_filtered=np.zeros((1, 100))

        self.kfx2=mykalman()
        self.kfy2=mykalman()
        self.kfz2=mykalman()
        self.fx2=np.zeros((1, 100))
        self.fy2=np.zeros((1, 100))
        self.fz2=np.zeros((1, 100))
        self.fx2_filtered=np.zeros((1, 100))
        self.fy2_filtered=np.zeros((1, 100))
        self.fz2_filtered=np.zeros((1, 100))

        self.fx_zero = 0#2.4543
        self.fy_zero = 0#2.444
        self.fz_zero = 0#2.454

    def initialize(self):
        if(self.ADC.ADS1263_init_ADC1('ADS1263_7200SPS') == -1):
            return False
        else:
            self.ADC.ADS1263_SetMode(0)
            return True

    def adc_raw(self,value):
        if(value >> 31 == 1):
            return -1*(REF*2 - value * REF / 0x80000000)
        else:
            return float((value * REF / 0x7fffffff))

    def acquire_data(self):
        ADC_Value = self.ADC.ADS1263_GetAll()    # get ADC1 value
        for i in range(0, 6):
            raw = self.adc_raw(ADC_Value[i])
            if i==0:
                self.fx1=(raw-self.fx_zero)*SCALE
                self.fx1_filtered=self.kfx1.update(self.fx1)
            if i==1:
                self.fy1=(raw-self.fy_zero)*SCALE
                self.fy1_filtered=self.kfy1.update(self.fy1)
            if i==2:
                self.fz1=(raw-self.fz_zero)*SCALE
                self.fz1_filtered=self.kfz1.update(self.fz1)
            if i==3:
                self.fx2=(raw-self.fx_zero)*SCALE
                self.fx2_filtered=self.kfx2.update(self.fx2)
            if i==4:
                self.fy2=(raw-self.fy_zero)*SCALE
                self.fy2_filtered=self.kfy2.update(self.fy2)
            if i==5:
                self.fz2=(raw-self.fz_zero)*SCALE
                self.fz2_filtered=self.kfz2.update(self.fz2)

    def publish_forces(self):
        self.acquire_data()
        forces1 = Float32MultiArray(data=[self.fx1_filtered,self.fy1_filtered,self.fz1_filtered])
        self.force1_pub.publish(forces1)
        forces2 = Float32MultiArray(data=[self.fx2_filtered,self.fy2_filtered,self.fz2_filtered])
        self.force2_pub.publish(forces2)

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
