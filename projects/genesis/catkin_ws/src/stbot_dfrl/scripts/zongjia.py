from mt_dxl import DxlAPI
from math import sin, cos, acos, asin, atan2, pi
import time
import numpy as np
import pandas as pd


def rad2value(rad):
    return int(rad / (2 * pi) * 4096 + 2047)


def mrad2value(rad):
    return int(-rad / (2 * pi) * 4096 + 2047)
# motor_group.torque_disable()

array = np.array(pd.read_csv('array.csv'))
MODE = 2
ADJUST = [0, -1024, -1024]
L1 = 40.0
L2 = 70.0
L3 = 95.0
L4 = 35.0
width = 40.0
height = 40.0
time_period = 4
t = np.linspace(0, 4, 401)
present_position = {'rf': np.array([(L1 + L2), -L4, L3]),
                    'rh': np.array([(L1 + L2), -L4, L3]),
                    'lf': np.array([(L1 + L2), -L4, L3]),
                    'lh': np.array([(L1 + L2), -L4, L3])}
motor_group = DxlAPI([1, 2, 3], '/dev/ttyUSB0', 4000000)
motor_group.set_operating_mode('p')
motor_group.torque_enable()
motor_group.set_position([2047, 0, 1024])
time.sleep(2)
print('ready to start!')
time.sleep(1)
print('start!')
if MODE == 1:
    for period in t:
        px = present_position['lf'][0]
        pz = (present_position['lf'][2] + 2 * width / time_period * period)
        py = present_position['lf'][1] - height / time_period ** 2 * period ** 2 + 2 * height / time_period * period

        theta1 = atan2(py, px) + asin(L4 / (px ** 2 + py ** 2) ** 0.5)
        theta3 = asin((2 * L1 * (cos(theta1) * px + sin(
            theta1) * py) - pz ** 2 - py ** 2 - px ** 2 + L2 ** 2 + L3 ** 2 + L4 ** 2 - L1 ** 2) / (2 * L2 * L3))
        theta2 = asin(pz / ((L2 - L3 * sin(theta3)) ** 2 + (L3 * cos(theta3)) ** 2) ** 0.5) - atan2(L3 * cos(theta3), L2 - L3 * sin(theta3))


        motor_group.set_position([rad2value(theta1), rad2value(theta2) + ADJUST[1], mrad2value(theta3) + ADJUST[2]])
        time.sleep(0.005)
    time.sleep(2)
if MODE == 2:
    rec = []
    for data in array:
        theta1 = data[0]
        theta2 = data[1]
        theta3 = data[2]
        motor_group.set_position([rad2value(data[0]), -rad2value(data[1]), rad2value(data[2])])
        time.sleep(0.01)
        px = L1 * cos(theta1) + L2 * cos(theta1) * cos(theta2) - L3 * cos(theta1) * sin(theta2+theta3) + L4 * sin(theta1)
        pz = L1 * sin(theta1) + L2 * sin(theta1) * cos(theta2) - L3 * sin(theta1) * sin(theta2+theta3) - L4 * cos(theta1)
        py = L2 * sin(theta2) + L3 * cos(theta2+theta3)
        rec.append([px, py, pz])
        print(px, py, pz)
    pd.DataFrame(np.array(rec)).to_csv('position_rec2.csv', index=False)

p_rec = []
if MODE == 3:
    motor_group.torque_disable()
    for i in range(500):
        p_rec.append(motor_group.get_position())
        time.sleep(0.01)
    p_rec = np.array(p_rec)
    print(p_rec)
    pd.DataFrame(np.array(p_rec)).to_csv('array.csv', index=False)

motor_group.torque_disable()

