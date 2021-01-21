#!/usr/bin/env python
# Software License Agreement (BSD License)
import rospy
import sys
from std_msgs.msg import Bool
sys.path.append("./")
import simRos
sys.path.append("/home/suntao/PycharmProjects/RL")
import vestiReflexRL


if __name__ == '__main__':
    arg = sys.argv[1:len(sys.argv)]
    print(arg)
    try:
        rosNode = simRos.simRosClass(arg)
        rl = vestiReflexRL.reflexRL()
        while not rospy.is_shutdown():
            termiState = rosNode.paraDir['/terminateController']==Bool(True)
            if termiState:
                break
            rl.setInput(rosNode.paraDir['/simTime'], rosNode.paraDir['/sensorValues'])
            rl.step()
            data = rl.getOutput()
            rosNode.setMotorPosition(data)
            rosNode.rosSpinOnce()

    except rospy.ROSInterruptException:
        pass
