
from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
from Haptic_Controller import CompensationController
import time
import numpy as np
import time
from Pong_Game import pong
import helper




robot = Robot.Robot("arm1")
udp = UDP(9876)
# ploter = PlotArm.PlotArm()
game = pong.pong()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65
STATUS = 38
TORQUE_CONTROL = 39


packet = 15 * [0, 0, 0]

Kv = np.matrix([[5, 0, 0], [0, -5, 0], [0, 0, -10]])
Kl = np.matrix([[25, 0, 0], [0, -10, 0], [0, 0, -50]])
controller = CompensationController.CompensationController(Kl, Kv)
pidConstants = [.0001, 0.0005, 0.001, 0.0001, 0.00025, 0.01, 0.0002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0]
upstream = udp.send_packet(0,TORQUE_CONTROL, pidConstants)
time0 = time.clock()


while (1):

    u = [0.0, 0.0, 0.0]
    robot.update(upstream)
    u = helper.remap(controller.getTorque(robot))
    dt = time.clock() - time0
    packet[0] = helper.angle_to_encoder(robot.qd[0]*dt + robot.q[0])
    packet[3] = helper.angle_to_encoder(robot.qd[1]*dt + robot.q[1])
    packet[6] = helper.angle_to_encoder(robot.qd[2]*dt + robot.q[2] + 0.5 * math.pi)
    packet[2] = u[0]
    packet[5] = u[1]
    packet[8] = u[2]
    pos0, pos1, pos2 = Dynamics.fk(robot)
    packet[9] = game.update(pos2[1])
    time0 = time.clock()
    upstream = udp.send_packet(0,37, packet)





