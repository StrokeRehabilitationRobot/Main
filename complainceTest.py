from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
from Haptic_Controller import GravityCompensationController
import time
import numpy as np
import time
robot = Robot.Robot("arm1")
ploter = PlotArm.PlotArm()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65

udp = UDP(9876)

packet = 15*[0,0,0]
sinWaveInc = 10;
sin_range = 400;
count = 0
Kv = np.matrix([ [.5,0,0],[0,-.5,0],[0,0,-.5]])
Kl = np.matrix([ [10,0,0],[0,-10,0],[0,0,-10]])
controller = GravityCompensationController.GravityCompensationController(Kl,Kv)
u = [0.0,0.0,0.0]
pidConstants = [.001, 0.0005, 0.01, 0, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
upstream = udp.send_packet(PID_CONFIG, pidConstants)
while(1):
    count +=1
    i = 0
    robot.update(upstream)
    u = controller.getTorque(robot)
    print robot.q[2]
    packet[0] = robot.angle_to_encoder(robot.q[0])
    packet[3] = robot.angle_to_encoder(robot.q[1])
    packet[6] = robot.angle_to_encoder(robot.q[2]+0.5*math.pi)

    packet[2] = u[0]
    packet[5] = u[1]
    packet[8] = u[2]
    packet[9] = 0

    upstream = udp.send_packet(37, packet)

    #upstream = udp.send_packet(37, packet)
    ploter.update(*Dynamics.fk(robot))
    #print robot.q






