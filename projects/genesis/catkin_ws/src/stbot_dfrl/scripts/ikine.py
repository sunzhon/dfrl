
    def __footTrajectory(self,SimulationTime, StepHeight, StepLength, period, dutyFactor):
        S = StepLength
        H = StepHeight
        T = period
        DF = dutyFactor
        Ty = (1 - DF) * T
        t1 = Ty
        t11 = Ty / 2
        t12 = Ty
        t = (round(SimulationTime * 100) % round(T * 100)) / 100.0

        if t < t1:
            Px = S * (t / Ty - 1 / 2.0 / math.pi * math.sin(2 * math.pi * t / Ty)) - S / 2.0
        else:
            Px = S / 2.0 - S * ((t - Ty) / (T - Ty) - 1 / 2.0 / math.pi * math.sin(2 * math.pi * (t - Ty) / (T - Ty)))

        if t < t11:
            Pz = 2 * H * (t / Ty - 1 / 4.0 / math.pi * math.sin(4 * math.pi * t / Ty))
        elif (t >= t11) and (t < t12):
           Pz = 2 * H * ((Ty - t) / Ty - 1 / 4.0 / math.pi * math.sin(4.0 * math.pi * (Ty - t) / Ty))
        else:
            Pz = 0.0

        Py = 0.0
        return [Px, Py, Pz]

    def gait(self,SimulationTime, StepHeight, StepLength, period, dutyFactor):

            footTrajectoryFR = self.__footTrajectory(SimulationTime, StepHeight, StepLength, period, dutyFactor)
            footTrajectoryRR = self.__footTrajectory(SimulationTime+period/2, StepHeight, StepLength, period, dutyFactor)
            footTrajectoryFL = self.__footTrajectory(SimulationTime+period/2, StepHeight, StepLength, period, dutyFactor)
            footTrajectoryRL = self.__footTrajectory(SimulationTime, StepHeight, StepLength, period, dutyFactor)

            output = footTrajectoryFR + footTrajectoryRR + footTrajectoryFL + footTrajectoryRL
            print(output)
            return output


    def ikinematic(self,gait):
        pass


