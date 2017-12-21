from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import time
import numpy as np
import csv
import helper
from Haptic_Controller import InverseDynamicsController

robot = Robot.Robot("arm1")
ploter = PlotArm.PlotArm()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65

udp = UDP(9876)

Kp = np.matrix([[.5, 0, 0], [0, 10, 0], [0, 0, 5]])
Kv = np.matrix([[5, 0, 0], [0, 20, 0], [0, 0, 5]])

control =  InverseDynamicsController.InverseDynamicsController(Kp,Kv)
data = []
# TODO make less shit
with open('DMPdata/test_joint_angles_left_modified.txt', 'r') as datafile:
    dd = csv.reader(datafile)
    for row in dd:
        g = row[0].split(" ")
        # print g[0]
        data.append((float(g[0]), float(g[1]), float(g[2])))
        # print data[0][0]
data1 = [data[uy][0] for uy in range(0, len(data))]
data2 = [data[uy][1] for uy in range(0, len(data))]
data3 = [data[uy][2] for uy in range(0, len(data))]

packet = 15*[0]

pidConstants = [0.001, 0.0005, .01, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0, ];
upstream = udp.send_packet(0,PID_CONFIG, pidConstants)

for theta1, theta2, theta3 in zip(data1,data2,data3):

    packet[0] = helper.angle_to_encoder(theta1-0.5*math.pi)
    packet[3] = helper.angle_to_encoder(theta2)
    packet[6] = helper.angle_to_encoder(theta3)
    packet[9] = 0
    upstream =  udp.send_packet(0,37,packet)
    robot.update(upstream)
    ploter.update(*Dynamics.fk(robot))







