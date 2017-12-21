from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import numpy as np
from Haptic_Controller import GravityCompensationController

import time

robot = Robot.Robot("arm1")
ploter = PlotArm.PlotArm()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65
STATUS = 38
TORQUE_CONTROL = 39

udp = UDP(9876)
baseConstants = [0.001, 0.0002, 0.01];
shoulderConstants = [0.002, 0.00025, 0.01];
elbowConstants = [0.002, 0.0004, 0.01];

packet = 15 * [0, 0, 0]
sinWaveInc = 10;
sin_range = 400;
count = 0
pidConstants = [0.001, 0.0005, .01, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
udp.send_packet(0,PID_CONFIG, pidConstants)
Kv = np.matrix([[.5, 0, 0], [0, -5, 0], [0, 0, -1]])
Kl = np.matrix([[1, 0, 0], [0, -10, 0], [0, 0, -50]])
controller = GravityCompensationController.GravityCompensationController(Kl, Kv)


def remap(val, OldMin, OldMax, ):
    NewMin = 0
    NewMax = 0.25
    OldRange = (OldMax - OldMin)
    NewRange = (NewMax - NewMin)
    NewValue = (((val - OldMin) * NewRange) / OldRange) + NewMin
    return abs(round(NewValue, 2))


while (1):
    count += 1

    u = controller.getTorque(robot)
    #print "link 1: " + str(u[1])
    #print "link 1: " + str(u[2])
    packet[0] = robot.angle_to_encoder(0)
    packet[3] = robot.angle_to_encoder(0)
    packet[6] = robot.angle_to_encoder(0.5*3.14)

    upstream = udp.send_packet(1,PID_CONTROL, packet)

    packet[0] = robot.angle_to_encoder(0)
    packet[3] = robot.angle_to_encoder(0)
    packet[6] = robot.angle_to_encoder(0.25*3.14)

    upstream = udp.send_packet(0, PID_CONTROL, packet)

    print upstream
    robot.update(upstream)
    # print robot.tau
    ploter.update(*Dynamics.fk(robot))
    # print robot.q






