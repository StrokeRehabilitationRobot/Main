from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import numpy as np
from Haptic_Controller import GravityCompensationController
import helper
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

packet = 15 * [0, 0, 0]

pidConstants = [0.001, 0.0005, .01, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
udp.send_packet(0,PID_CONFIG, pidConstants)
k = np.matrix([[0, 0, 0], [0, 0.04, 0], [0, 0, 0.005]])
controller = GravityCompensationController.GravityCompensationController(k)


def remap(val, OldMin, OldMax, ):
    NewMin = 0
    NewMax = 2.5
    OldRange = (OldMax - OldMin)
    NewRange = (NewMax - NewMin)
    NewValue = (((val - OldMin) * NewRange) / OldRange) + NewMin
    return abs(round(NewValue, 2))

while (1):
    packet = 15 * [0.0]
    u = controller.getTorque(robot)
    print "link 1 raw: " + str(u[1])
    print "link 2 raw: " + str(u[2])
    packet[5] = remap(.04*u[1], 0.001, 0.97)
    packet[8] = remap(.005*u[2], 0.001, 0.45)
    packet[9] = 0
    print "link 1 remap: " + str(packet[5])
    print "link 2 remap: " + str(packet[8])

    upstream = udp.send_packet(0,TORQUE_CONTROL, packet)

    robot.update(upstream)
    # print robot.tau
    ploter.update(*Dynamics.fk(robot))
    # print robot.q
