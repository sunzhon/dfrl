import rospy
import sys

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock



class simRosClass:

    def simulationTimeCallback(self, simTime):
        self.paraDir[self.simTimeTopic] = simTime

    def terminateNodeCallback(self, termNode):
        self.paraDir[self.terminateNodeTopic] = termNode

    def startSimCallback(self, startSim):
        self.paraDir[self.startSimTopic] = startSim

    def simulationStepDoneCallback(self, simStepDone):
        self.paraDir[self.simStepDoneTopic] = simStepDone

    def simulationStateCallback(self, state):
        self.paraDir[self.simStateTopic] = state

    def sensorValueCallback(self, sensorValue):
        self.paraDir[self.sensorValueTopic] = sensorValue

    def __init__(self, arg):
        if len(arg) > 12:
            self.simTimeTopic = "/" + arg[1]
            self.terminateNodeTopic = "/" +arg[2]
            self.startSimTopic = "/" + arg[3]
            self.pauseSimTopic = "/" + arg[4]
            self.stopSimTopic = "/" + arg[5]
            self.enableSyncModeTopic = "/" + arg[6]
            self.triggerNextStepTopic = "/" + arg[7]
            self.simStepDoneTopic = "/" + arg[8]
            self.simStateTopic = "/" + arg[9]
            self.sensorValueTopic = "/" + arg[11]
            self.reflexMotorTopic = "/" + arg[12]
            self.rosRate =int(arg[13])
            self.jointNumber = int(arg[15])

            self.paraDir = {self.simTimeTopic: 0.0, self.terminateNodeTopic: Bool(False), self.startSimTopic: Bool(False),
                       self.pauseSimTopic: Bool(False), self.stopSimTopic: Bool(False), self.enableSyncModeTopic: Bool(False),
                       self.triggerNextStepTopic: 0.0, self.simStepDoneTopic: 0.0,
                       self.simStateTopic: 0.0, self.sensorValueTopic:0.0, self.reflexMotorTopic: 0.0}
        else:
            print("reflex node parameters error")
            exit()
        #rospy.Subscriber(self.simTimeTopic, Clock, self.simulationTimeCallback, queue_size=1)
        rospy.Subscriber(self.terminateNodeTopic, Bool, self.terminateNodeCallback, queue_size=1)
        #rospy.Subscriber(self.simStepDoneTopic, Bool, self.simulationStepDoneCallback, queue_size=1)
       # rospy.Subscriber(self.simStateTopic, Int32, self.simulationStateCallback, queue_size=1)
        rospy.Subscriber(self.sensorValueTopic, Float32MultiArray, self.sensorValueCallback, queue_size=1)
        # startSimPub = rospy.Publisher(startSimTopic, Bool, queue_size=1)
        # pauseSimPub = rospy.Publisher(pauseSimTopic, Bool, queue_size=1)
        # stopSimPub = rospy.Publisher(stopSimTopic, Bool, queue_size=1)
        # enableSyncModePub = rospy.Publisher(enableSyncModeTopic, Bool, queue_size=1)
        # startSimPub = rospy.Publisher(triggerNextStepTopic, Bool, queue_size=1)
        self.reflexMotor = Float32MultiArray()
        self.reflexMotorpub = rospy.Publisher(self.reflexMotorTopic, Float32MultiArray, queue_size=1)
        rospy.init_node('Reflex', anonymous=False)
        self.rate = rospy.Rate(self.rosRate)  # run as soon as faster
        #print("reflex ros node rate :%d " % rosRate)
        print("reflex node initial succussfully!")

    def rosSpinOnce(self):
        self.rate.sleep()
        """
        rateMet = self.rate.sleep()
        while not rateMet:
            rateMet = self.rate.sleep()
        """


# publish
    def setMotorPosition(self, arrayData):
        #  publish the motor positions:
        if isinstance(arrayData, list) and (len(arrayData) == self.jointNumber):
            for val in arrayData:
                self.reflexMotor.data.append(val)
            self.reflexMotorpub.publish(self.reflexMotor)
            del self.reflexMotor.data[:]
        else:
            print("publish data type is error")

    def __del__(self):
        rospy.INFO("shutdown this reflex node")
        rospy.signal_shutdown("The simulation is shutdown ")


if __name__ == '__main__':
    arg = sys.argv[1:len(sys.argv)]
    print(arg)
    try:
        simRosClass(arg)
    except rospy.ROSInterruptException:
        pass

