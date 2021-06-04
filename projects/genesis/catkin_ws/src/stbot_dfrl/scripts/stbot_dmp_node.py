#!/usr/bin/env python
# Software License Agreement (BSD License)
import sys
import time
from std_msgs.msg import Bool
#sys.path.append("/home/suntao/PycharmProjects/Reflex")
#sys.path.append("/home/suntao/workspace/PythonProjects/PyPro3/DMP/pydmps/pydmps/")
import sys
from os.path import abspath, join, dirname
print(__file__)
sys.path.insert(0, join(abspath(dirname(__file__)), 'pydmps'))
print(sys.path)
from imitate_so2CPG import dmpSO2CPG
import simRos
import rospy
import numpy as np


if __name__ == '__main__':
    time.sleep(3) 
    arg = sys.argv[1:len(sys.argv)]
    print(arg)
    try:
        rosNode = simRos.simRosClass(arg)
        dmp = dmpSO2CPG(n_dmps=2,dt=0.01,tau=6,n_bfs=200)
        while not rospy.is_shutdown():
            termiState = rosNode.paraDir[rosNode.terminateNodeTopic]==Bool(True)
            if termiState:
                break
            #dmp.setInput(rosNode.paraDir[rosNode.simTimeTopic], rosNode.paraDir[rosNode.sensorValueTopic])
            dmp.step()
            dmp.step()
            dmp.step() # step 3 times so that speed up the locomotion
            data=[0,0,0 ,0,0,0, 0,0,0, 0,0,0]
            data[1]=dmp.getOutput(0)-0.2
            data[2]=dmp.getOutput(1)-0.2

            data[4]=-dmp.getOutput(0)-0.2
            data[5]=-dmp.getOutput(1)-0.2

            data[7]=-dmp.getOutput(0)-0.2
            data[8]=-dmp.getOutput(1)-0.2

            data[10]=dmp.getOutput(0)-0.2
            data[11]=dmp.getOutput(1)-0.2
            data=[0.12*dd for dd in data]
            #data=4*np.tanh(data)
            rosNode.setMotorPosition(data)
            rosNode.rosSpinOnce()

    except rospy.ROSInterruptException:
        pass
