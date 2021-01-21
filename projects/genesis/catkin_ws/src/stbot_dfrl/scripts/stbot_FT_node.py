#!/usr/bin/env python
# Software License Agreement (BSD License)
import sys
import time
from std_msgs.msg import Bool
import eight_shape2
import inverseKinematics
import simRos
import rospy


if __name__ == '__main__':
    time.sleep(3) 
    arg = sys.argv[1:len(sys.argv)]
    print(arg)
    try:
        rosNode = simRos.simRosClass(arg)
        ft = eight_shape2.FootTrajectory()
        ik = inverseKinematics.IKinematic()
        while not rospy.is_shutdown():
            termiState = rosNode.paraDir[rosNode.terminateNodeTopic]==Bool(True)
            if termiState:
                break
            ft.step()
            ik.setInput(ft.getOutput())
            ik.step()
            data = ik.getOutput();
            rosNode.setMotorPosition(data)
            rosNode.rosSpinOnce()

    except rospy.ROSInterruptException:
        pass
