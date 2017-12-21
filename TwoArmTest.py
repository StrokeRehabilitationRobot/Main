from UDP.UDP import UDP
import math
import Robot
import helper
import PlotArm
from Dynamics import Dynamics
import numpy as np
from Haptic_Controller import GravityCompensationController

import time

thanos = Robot.Robot("thanos")
helena = Robot.Robot("helena")


udp = UDP(9876)

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65
STATUS = 38
TORQUE_CONTROL = 39




packet = 15 * [0, 0, 0]
def remap(val, OldMin, OldMax, ):
    NewMin = 0
    NewMax = 0.25
    OldRange = (OldMax - OldMin)
    NewRange = (NewMax - NewMin)
    NewValue = (((val - OldMin) * NewRange) / OldRange) + NewMin
    return abs(round(NewValue, 2))


def move(inc):
    packet[0] = helper.angle_to_encoder(0)
    packet[3] = helper.angle_to_encoder(0)
    packet[6] = helper.angle_to_encoder(inc * 3.14)

    upstream = udp.send_packet(thanos._id, PID_CONTROL, packet)
    thanos.update(upstream)

    msg = helper.make_packet(thanos.q, thanos.qd, thanos.tau)
    upstream = udp.send_packet(helena._id, PID_CONTROL, msg)
    helena.update(upstream)


#move the arm in a wave
while (1):


    for i in np.linspace(0, 1, num=50):
        move(i)

    for i in np.linspace(1, 0, num=50):
        move(i)











