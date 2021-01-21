#! /usr/bin/env python

# -*- coding:utf-8 -*-
from  matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import pandas as pd
class FootTrajectory:
    index=0
    Py0=0.0
    Px0=0.12
    Pz0=0.01

    def __init__(self):
        dist1 = 0.06
        dist11 = 0.1
        dist12 = 0.1
        dist2 = 0.06
        dist3 = 0.03

        '''
        #formula eight shape trajectory version 1.0
        num_targets=400
        self.num_targets=num_targets
        target_position=np.array(
                [[
                dist2*np.sin(theta)*np.cos(theta),
                dist2*np.cos(theta),
                dist3*np.sin(theta)
                ]
                for theta in np.linspace(0,np.pi*2.0,num_targets)]
                )
        plt.subplot(1,2,1);
        plt.plot(target_position[:,0]);
        plt.plot(target_position[:,1]);
        plt.subplot(1,2,2);
        plt.plot(target_position[:,0],target_position[:,1]);
        plt.show();
        '''

        '''
        # formula eight shape trajectory version 2.0
        num_targets=400
        self.num_targets=num_targets
        xt,yt,zt=[],[],[]
        for theta in np.linspace(0,np.pi*2.0,num_targets):
            #xt.append(dist1*np.sin(theta)*np.cos(theta))
            xt.append(dist1*np.sin(theta*2.0))
            yt.append(dist2*np.sin(theta))
            zt.append(0.0*dist3*np.cos(theta))
            #dist1 = dist11*yt[-1] if yt[-1] > 0.0 else dist12*yt[-1]

        self.x=zt+self.Px0*np.ones(len(yt))
        self.y=xt+self.Py0*np.ones(len(xt))
        self.z=yt+self.Pz0*np.ones(len(zt))

        '''
        # data from file for eight shape trajectory version 3.0
        data_file = '/home/suntao/workspace/gorobots/projects/stbot/genesis/catkin_ws/src/stbot/scripts/eightTra.csv'
        #data_file = '/home/suntao/workspace/gorobots/projects/stbot/genesis/catkin_ws/src/stbot/scripts/parabolaTra.csv'
        #data_file = './eightTra.csv'
        resource_data = pd.read_csv(data_file, sep=',', names=['x','y'])


        data = np.zeros(resource_data.shape)
        num_targets=len(data)
        self.num_targets=num_targets
        for index in range(2):
            temp = resource_data.iloc[:,index]
            data[:,index]=tuple(temp)
        self.z=data[:,0]/100.0 + self.Pz0
        self.y=data[:,1]/35.0 + self.Py0 # taitui gaodu
        self.x=np.zeros(self.y.shape) +self.Px0
        '''
        # data from file for parabola shape trajectory version 3.0
        data_file = '/home/suntao/workspace/gorobots/projects/stbot/genesis/catkin_ws/src/stbot/scripts/parabolaTra.csv'
        resource_data = pd.read_csv(data_file, sep=',', names=['num','x','y'])


        data = np.zeros(resource_data.shape)
        num_targets=len(data)
        self.num_targets=num_targets
        for index in range(3):
            temp = resource_data.iloc[:,index]
            data[:,index]=tuple(temp)
        self.z=data[:,1]/100.0 + self.Pz0
        self.y=data[:,2]/100.0 + self.Py0 # taitui gaodu
        self.x=np.zeros(self.y.shape) +self.Px0
        '''
    def step(self):
        if self.index<=self.num_targets-2:
            self.index=self.index+1
        else:
            self.index=0

    def getOutput(self):
        return [self.x[self.index],self.y[self.index],self.z[self.index]]

if __name__=="__main__":
    FT = FootTrajectory()
    x,y,z = [],[],[]
    for idx in range(len(FT.x)):
        x.append(FT.getOutput()[0])
        y.append(FT.getOutput()[1])
        z.append(FT.getOutput()[2])
        FT.step()
    plt.subplot(1,2,1);
    plt.plot(x,'r');
    plt.plot(y,'b');
    plt.plot(z,'k');
    plt.subplot(1,2,2);
    plt.plot(y,z);
    fig=plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(x,y,z,'g');
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    plt.show();
