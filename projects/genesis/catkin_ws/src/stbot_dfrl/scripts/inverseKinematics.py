#! /usr/bin/env python
# -*- coding:utf-8- -*-

import sys
import math
from math import sin, cos, acos, asin, atan2, pi
import cmath
import numpy as np

'''
class DelayLine:
    def __init__(self,count):
        self.step=0
        self.buffer=np.zeros(count)

    def Write(self, value)
        self.buffer[step]=value

    def Read(self,index):
        return self.buffer[self.Mod((self.step-index),self.buffer.len())]

    def Step(self):
        slef.step++
        if(self.step%self.buffer.len()==0):
            self.step=0
    def Mod(self, x,m):
        r=x%m
        if r<0:
            return r+m
        else:
            return r
'''

class IKinematic:
    def __init__(self):
        self.link=[0.07,0.085]
        #self.theta_init=[80.0/180*np.pi, 1.055-0.2,-92.0/180*np.pi]
        self.theta_init=[80.0/180*np.pi, 0.8, 0.5]

        self.input=[0,0]
        self.output= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0    ]
    def __del__(self):
        del self.input,self.output
    
    def step(self):
        '''
        theta=[0,0]
        square1=self.input[0]**2+self.input[1]**2
        square2=self.link[0]**2+self.link[1]**2       
        divide=(square1-square2)/(2.0*self.link[0]*self.link[1])
        theta[1]=math.acos(divide);

        temp1=-self.link[0]-self.link[1]*np.cos(theta[1])
        temp2=self.link[1]*np.sin(theta[1])
        temp3=self.input[0]/cmath.sqrt(temp1**2+temp2**2)
        theta[0]=math.asin(temp3)-math.atan(temp2/temp1)

        theta[0]=-theta[0] + self.theta_init[0]
        theta[1]=theta[1] + self.theta_init[1]

        self.output=[0.0, theta[0],theta[1]]*4
        '''
        px=self.input[0]
        py=self.input[1]
        pz=self.input[2]
        
        L1 = 0.040
        L2 = 0.070
        L3 = 0.095
        L4 = 0.035
        theta1 = atan2(py, px) + asin(L4 / (px ** 2 + py ** 2) ** 0.5)+self.theta_init[0]
        theta3 = asin((2 * L1 * (cos(theta1) * px + sin(
                        theta1) * py) - pz ** 2 - py ** 2 - px ** 2 + L2 ** 2 + L3 ** 2 + L4 ** 2 - L1 ** 2) / (2 * L2 * L3))+self.theta_init[2]
        theta2 = asin(pz / ((L2 - L3 * sin(theta3)) ** 2 + (L3 * cos(theta3)) ** 2) ** 0.5) - atan2(L3 * cos(theta3), L2 - L3 * sin(theta3))+self.theta_init[1]

        self.output=[theta1-self.theta_init[0]-20.0/180*np.pi,theta2,theta3]*4
        self.output[6]=-1.0*self.output[6]
        self.output[9]=-1.0*self.output[9]

    def getOutput(self):
        return self.output

    def setInput(self,inputTra):
        self.input=inputTra

if __name__=="__main__":
        ik=Ikinematics()

