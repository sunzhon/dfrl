#!/usr/bin/env python
# -*- coding:utf-8 -*-
import sys
import math

class attitudeReflexClass:

    def __init__(self):

        self.input= []
        self.valJ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0]
        self.output = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0]
        self.time =0.0
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.footSt={'RF':0.0,'RH':0.0,'LF':0.0,'LH':0.0}
        self.jointGain={'x':1.5,'y':1.5}
        self.posGain = {'x':2.0,'y':2.5}
        self.moveSt=0.0

    def __del__(self):
        del self.input,self.output, self.footSt,self.jointGain,self.posGain
        
         
    def setInput(self,input1,input2):
        self.time=input1
        if not isinstance(input2,float):
            sdata=input2.data
            self.footSt['RF']= 0.0 if sdata[12] >0.0 else 1.0
            self.footSt['RH']= 0.0 if sdata[13] >0.0 else 1.0
            self.footSt['LF']= 0.0 if sdata[14] >0.0 else 1.0
            self.footSt['LH']= 0.0 if sdata[15] >0.0 else 1.0
            if self.footSt['RF']>0.0 and self.footSt['LH'] >0.0:
                self.moveSt=1.0
            else:
                self.moveSt=0.0

            self.roll = sdata[16]
            self.pitch = sdata[17]
            self.yaw = sdata[18]
            #print("   roll:", self.roll)
            #print("  pitch:", self.pitch)
            #print("foot ST:", self.footSt['RF'])

    
    def balanceStrategy(self):
        mPos={}
        if self.roll > 0.06:
            rollSt=0.04
        elif self.roll <-0.06:
            rollSt=-0.04
        else:
            rollSt=0.0

        if self.pitch > 0.06:
            pitchSt=0.04
        elif self.pitch <-0.06:
            pitchSt=-0.04
        else:
            pitchSt=0.0

        mPos['y']= self.moveSt * self.posGain['y'] * rollSt
        mPos['x']= self.moveSt * self.posGain['x'] * pitchSt
        #print("Pos[y]:", mPos['y'])
        return mPos
        

    def moveCog(self,cogPos):
        gain=0.8
        # along with x aix
        RFx = gain*math.tanh(self.jointGain['x'] * self.footSt['RF']*cogPos['x'])
        RHx = gain*math.tanh(self.jointGain['x'] * self.footSt['RH']*cogPos['x'])
        LFx = gain*math.tanh(self.jointGain['x'] * self.footSt['LF']*cogPos['x'])
        LHx = gain*math.tanh(self.jointGain['x'] * self.footSt['LH']*cogPos['x'])

        # along with y aix
        RFy = gain*math.tanh(self.jointGain['y'] * self.footSt['RF'] * cogPos['y'])
        RHy = gain*math.tanh(self.jointGain['y'] * self.footSt['RH'] * cogPos['y'])
        LFy = gain*math.tanh(-self.jointGain['y'] * self.footSt['LF'] * cogPos['y'])
        LHy = gain*math.tanh(-self.jointGain['y'] * self.footSt['LH'] * cogPos['y'])
        self.valJ = [RFy, RFx, 0.0, RHy, RHx, 0.0, LFy, LFx, 0.0, LHy, LHx, 0.0]
        for idx,val in enumerate(self.valJ):
            self.output[idx]= val if abs(val) < 0.4 else val/abs(val)*0.4

        #print("   RFx:%1.3f" %self.output[1])
        self.output = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, .0,.0, 0.0, .0, .0, 0.0]

    def step(self):
        inputMov=self.balanceStrategy()
        self.moveCog(inputMov)

    def getOutput(self):
        return self.output




if __name__ == '__main__':
    arg = sys.argv[1:len(sys.argv)]
    print(arg)
    try:
        ar=attitudeReflexClass()
        inputdata=[0.0,0.0,0.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0,.0]
        print(len(inputdata))
        ar.setInput(2.0,inputdata)
        ar.step()
    except IOError:
        pass
