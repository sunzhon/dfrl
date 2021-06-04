
"""
Copyright (C) 2013 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

from pydmps.dmp import DMPs

import numpy as np

import sys
import matplotlib.pyplot as plt

#sys.path.append("/home/suntao/workspace/PythonProjects/PyPro3/DMP/pydmps/pydmps/")
#sys.path.append("/home/suntao/workspace/PythonProjects/PyPro3/Oscillator/")

from dmp_rhythmic import DMPs_rhythmic 

from So2Oscillator import So2Oscillator
from So2Oscillator import PCPG

# ==============================
# Test code
# ==============================

class dmpSO2CPG():
    def __init__(self,n_dmps=2,dt=0.01,tau=1,n_bfs=1000,cpg_mi=0.08):
    # SO2 CPG
        step_nums=600
        so2=So2Oscillator(cpg_mi);
        out1=[]
        out2=[]
        for idx in range(step_nums):
            so2.step()
            out1.append(so2.getOutput(1))
            out2.append(so2.getOutput(2))
        
        # remove the initial period to get stable output
        out1=out1[100:]
        out2=out2[100:]
        if False:
            plt.figure(1, figsize=(6, 3))
            plt.plot(out1)
            plt.plot(out2)
            plt.title("SO 2 CPG")
            plt.xlabel("time (ms)")
            plt.ylabel("SO2 CPG outputs")
            plt.legend(["O1", "O2"], loc="lower right")
            plt.tight_layout()
            plt.plot(pout1)

        #transfer to step wave
        pout1=[]
        for ii, value in enumerate(out1):
            if value>0.8:
                pout1.append(1)
            else:
                pout1.append(0)
        # find a period of the CPG output
        pout1_array=np.array(pout1)
        max_index_1=np.where(pout1_array==np.max(pout1_array))[0]
        points_key=[]
        for idx in range(len(max_index_1)-1):
            if(max_index_1[idx+1]-max_index_1[idx]) >4:
                points_key.append(max_index_1[idx+1])
        out1=out1[points_key[1]:points_key[2]+1]
        out2=out2[points_key[1]:points_key[2]+1]
        self.cpg_o1=out1
        self.cpg_o2=out2

        timesteps=int(2*np.pi/dt)
        # DMP rhythmic
        self.tau=tau
        self.dmp = DMPs_rhythmic(dt=dt,n_dmps=n_dmps, n_bfs=n_bfs)
        self.dmp.imitate_path(y_des=np.array([out1, out2]),plot=False)
        #y_track, dy_track, ddy_track = dmp.rollout()

        self.y_track=[]
        self.dy_track=[]
        self.ddy_track=[]


    def setTau(self,tau):
        self.tau=tau

    def step(self):
        self.y_track, self.dy_track, self.ddy_track=self.dmp.step(tau=self.tau)

    def getOutput(self,idx):
        return self.y_track[idx]

    def getCPGOutput(self):
        return [self.cpg_o1, self.cpg_o2]



if __name__ == "__main__":
    n_dmps=2
    dt=0.02
    tau=1
    dmp_cpg=dmpSO2CPG(n_dmps=n_dmps,dt=dt,n_bfs=200,tau=tau,cpg_mi=0.08)
    timesteps=int(2*np.pi/dt)
    print(timesteps)
    #y_track, dy_track, ddy_track = dmp.rollout()
    y_track = np.zeros((timesteps, n_dmps))
    dy_track = np.zeros((timesteps, n_dmps))
    ddy_track = np.zeros((timesteps, n_dmps))
    for t in range(timesteps):
        #run and record timestep
        dmp_cpg.step()
        y_track[t,0] = dmp_cpg.getOutput(0)
        y_track[t,1] = dmp_cpg.getOutput(1)


    # plot  training
    plt.figure(2,figsize=(5,10))

    #gs1=gridspec.GridSpec(2,1)#13
    #gs1.update(hspace=0.18,top=0.95,bottom=0.16,left=0.12,right=0.98)
    #axs=[]
    #axs.append(fig.add_subplot(gs1[0:1,0]))
    #axs.append(fig.add_subplot(gs1[1:2,0]))




    plt.subplot(211)
    #plt.tight_layout()

    time=np.linspace(0, 2*np.pi, timesteps)
    plt.plot(time,y_track[:,0],'r')
    plt.plot(time,y_track[:,1],'b')
    cpg_o1=dmp_cpg.getCPGOutput()[0]
    cpg_o2=dmp_cpg.getCPGOutput()[1]
    print(len(cpg_o2))
    time=np.linspace(0, 2*np.pi, len(cpg_o1))
    plt.plot(time,cpg_o1,'r--',lw=2)
    plt.plot(time,cpg_o2,'b--',lw=2)
    plt.legend([u'DMP $O_1$',u'DMP $O_2$',u'SO (2) CPG $O_1$', u'SO (2) CPG $O_2$'],ncol=2,loc='best')
    plt.ylabel("Outputs")
    plt.grid()
    plt.xlabel("Phase [rad]")

    plt.title('CPG MI=0.08')
    plt.subplot(212)
    plt.axis('equal')
    plt.grid()
    plt.plot(y_track[:,0],y_track[:,1],'r')
    plt.plot(cpg_o1,cpg_o2,'b--')
    plt.ylabel(u"$O_2$")
    plt.xlabel(u"$O_1$")
    plt.show()


    # plot scaling
    dmp_cpg.setTau(3)
    timesteps=int(2*np.pi/dt)
    print(timesteps)
    #y_track, dy_track, ddy_track = dmp.rollout()
    y_track = np.zeros((timesteps, n_dmps))
    dy_track = np.zeros((timesteps, n_dmps))
    ddy_track = np.zeros((timesteps, n_dmps))
    for t in range(timesteps):
        #run and record timestep
        dmp_cpg.step()
        y_track[t,0] = dmp_cpg.getOutput(0)
        y_track[t,1] = dmp_cpg.getOutput(1)

    plt.figure()
    time=np.linspace(0, 2*np.pi, timesteps)
    plt.plot(time,y_track[:,0])
    plt.plot(time,y_track[:,1])
    plt.show()


    

if __name__ == "__test__":

    # SO2 CPG
    step_nums=600
    so2=So2Oscillator(0.08);
    out1=[]
    out2=[]
    for idx in range(step_nums):
        so2.step()
        out1.append(so2.getOutput(1))
        out2.append(so2.getOutput(2))
        
    # remove the initial period to get stable output
    out1=out1[100:]
    out2=out2[100:]
    if False:
        plt.figure(1, figsize=(6, 3))
        plt.plot(out1)
        plt.plot(out2)
        plt.title("SO 2 CPG")
        plt.xlabel("time (ms)")
        plt.ylabel("SO2 CPG outputs")
        plt.legend(["O1", "O2"], loc="lower right")
        plt.tight_layout()
        plt.plot(pout1)

    #transfer to step wave
    pout1=[]
    for ii, value in enumerate(out1):
        if value>0.8:
            pout1.append(1)
        else:
            pout1.append(0)


    
    pout1_array=np.array(pout1)
    max_index_1=np.where(pout1_array==np.max(pout1_array))[0]
    points_key=[]
    for idx in range(len(max_index_1)-1):
        if (max_index_1[idx+1]-max_index_1[idx]) >4:
            points_key.append(max_index_1[idx+1])
    out1=out1[points_key[1]:points_key[2]]
    out2=out2[points_key[1]:points_key[2]]
    timesteps=628;#len(out1)

    # test imitation of path run
    plt.figure(2, figsize=(6, 4))
    n_bfs = [1000]
    
    n_dmps=2
    for ii, bfs in enumerate(n_bfs):
        dmp = DMPs_rhythmic(n_dmps=n_dmps, n_bfs=bfs)
        dmp.imitate_path(y_des=np.array([out1, out2]),plot=False)
        #y_track, dy_track, ddy_track = dmp.rollout()
        y_track = np.zeros((timesteps, n_dmps))
        dy_track = np.zeros((timesteps, n_dmps))
        ddy_track = np.zeros((timesteps, n_dmps))
        for t in range(timesteps):
            # run and record timestep
            y_track[t], dy_track[t], ddy_track[t] = dmp.step()


        plt.figure(2)
        plt.subplot(211)
        plt.plot(np.linspace(0,2*np.pi,timesteps), y_track[:, 0], lw=2)
        plt.subplot(212)
        plt.plot(np.linspace(0,2*np.pi,timesteps),y_track[:, 1], lw=2)

    plt.subplot(211)
    a = plt.plot(np.linspace(0,2*np.pi,len(out1)),out1, "r--", lw=2)
    plt.title("DMP imitate path")
    plt.xlabel("time (ms)")
    plt.ylabel("system trajectory")
    plt.legend([a[0]], ["desired path"], loc="lower right")
    plt.subplot(212)
    b = plt.plot(np.linspace(0,2*np.pi,len(out1)),out2, "r--", lw=2)
    plt.title("DMP imitate path")
    plt.xlabel("time (ms)")
    plt.ylabel("system trajectory")
    plt.legend(["%i BFs" % i for i in n_bfs], loc="lower right")
    plt.tight_layout()
    plt.show()

    dmp.save_weights("weights_file.csv")
    np.savetxt("o1.txt",y_track)



    #y_track, dy_track, ddy_track = dmp.rollout()
    y_track = np.zeros((timesteps*4, n_dmps))
    dy_track = np.zeros((timesteps*4, n_dmps))
    ddy_track = np.zeros((timesteps*4, n_dmps))
    for t in range(timesteps*4):
        # run and record timestep
        y_track[t], dy_track[t], ddy_track[t] = dmp.step()

    plt.plot(y_track[:,0])
    plt.plot(y_track[:,1])
    plt.show()
