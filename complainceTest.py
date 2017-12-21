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
    packet = 15 * [0, 0, 0]
    u = controller.getTorque(robot)

    packet[0] = helper.angle_to_encoder(0)
    packet[3] = helper.angle_to_encoder(0)
    packet[6] = helper.angle_to_encoder(0.5*3.14)
    packet[5] = remap(0.5*u[1], 0.001, 0.97)
    packet[8] = remap(0.25*u[2], 0.001, 0.45)
    packet[9] = 0
    print "link 1 remap: " + str(packet[5])
    print "link 2 remap: " + str(packet[8])

    upstream = udp.send_packet(1,TORQUE_CONTROL, packet)
    print upstream
    robot.update(upstream)
    # print robot.tau
    ploter.update(*Dynamics.fk(robot))
    # print robot.q






