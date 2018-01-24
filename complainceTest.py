from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import numpy as np
from Haptic_Controller import GravityCompensationController
import helper
import time
from Haptic_Controller import CompensationController

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

pidConstants = [0.001, 0.0005, .01, 0.002, 0.0025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
udp.send_packet(0,PID_CONFIG, pidConstants)
k = np.matrix([[0, 0, 0], [0, 0.4, 0], [0, 0, 0.5]])
Kv = np.matrix([[0, 0, 0], [0, -5, 0], [0, 0, -10]])
Kl = np.matrix([[0.1, 0, 0], [0, -1, 0], [0, 0, -50]])
Kg = np.matrix([[0, 0, 0], [0, 1.3, 0], [0, 0, 0.005]])
controller = CompensationController.CompensationController(Kl, Kv, Kg)


while (1):
    packet = 15 * [0.0]
    u = helper.remap(controller.getTorque(robot))
    tau = []
    if Dynamics.fk(robot)[2][1] < -0.15:
        tau = Dynamics.get_J_tranpose(robot)*np.matrix( [ [-1],[0],[0] ] )
        print tau
        u[0] = u[0] +tau[0]

    packet[2] = u[0]
    packet[5] = u[1]
    packet[8] = u[2]
    packet[9] = 0

    upstream = udp.send_packet(0,TORQUE_CONTROL, packet)

    robot.update(upstream)
    # print robot.tau
    ploter.update(*Dynamics.fk(robot))
    # print robot.q
