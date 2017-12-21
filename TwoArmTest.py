from UDP.UDP import UDP
import math
import Robot
import helper
import PlotArm
from Dynamics import Dynamics
import numpy as np
from Haptic_Controller import GravityCompensationController

import time

thanos = Robot.Robot("thanos",id=0)
helena = Robot.Robot("helena",id=1)
ploter = PlotArm.PlotArm()

udp = UDP(9876)

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65
STATUS = 38
TORQUE_CONTROL = 39


pidConstants = [0.001, 0.0005, .01, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
udp.send_packet(0,PID_CONFIG, pidConstants)
udp.send_packet(1,PID_CONFIG, pidConstants)


packet = 15 * [0.0, 0.0, 0.0]


def move(inc):
    packet[0] = helper.angle_to_encoder(math.pi*inc)
    packet[3] = helper.angle_to_encoder(0)
    packet[6] = helper.angle_to_encoder(0.5*math.pi)

    upstream = udp.send_packet(helena._id, PID_CONTROL, packet)
    helena.update(upstream)
    ploter.update(*Dynamics.fk(helena))
    print "desired: " + str(packet)
    print "angle: " + str(helena.q[0])
    print "angle: " + str(helena.q[1])
    print "angle: " + str(helena.q[2])
    msg = helper.make_packet([helena.q[0],0,helena.q[2]], [0,0,0], [0,0,0])
    print "msg: " + str(msg)

    upstream = udp.send_packet(thanos._id, PID_CONTROL,msg)
    thanos.update(upstream)



#move the arm in a wave
while (1):


    for i in np.linspace(0, 0.5, num=100):
        move(i)

    for i in np.linspace(0.5, 0, num=100):
        move(i)











