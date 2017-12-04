from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import numpy as np
import time


robot = Robot.Robot()
ploter = PlotArm.PlotArm()
time.sleep(2)
VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65

udp = UDP(9876)

pidConstants = [0.001, 0.0002, .01, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0, ];
upstream = udp.send_packet(PID_CONFIG, pidConstants)

packet = 15*[0,0,0]
sinWaveInc = 10
sin_range = 400

#while(1):
#
#     # for k in xrange(sinWaveInc):
#     #     incremtal = float(k / sinWaveInc);
#     #
#     #     for j in xrange(4):
#     #         packet[(j * 3) + 0] = (math.sin(incremtal * math.pi * 2.0) * sin_range) + (sin_range);
#     #         packet[(j * 3) + 1] = 0
#     #         packet[(j * 3) + 2] = 3
# #
#     packet[0] = robot.angle_to_encoder(0)
#     packet[3] = robot.angle_to_encoder(0)
#     packet[6] = robot.angle_to_encoder(0)
#     print packet[0]
#     print packet[3]
#     print packet[6]
#     upstream =  udp.send_packet(37,packet)
#     robot.update(upstream)
#     ploter.update(*Dynamics.fk(robot))
#     print robot.q


#
a, b, c,d = Dynamics.trajectory([0,0.5*math.pi], [ 0,0 ], 5)
print (a, b, c,d)

start = time.time()
current = start
dt = current-start
while dt < 5:
    q = a + b*dt +  c*dt*dt + d*dt*dt*dt
    qd = b +  0.5*c*dt + (1/3.)*d*dt*dt
    packet[3] = robot.angle_to_encoder(q[0])
    packet[4] = robot.angle_to_encoder(qd[0])
    packet[5] = .3
    upstream =  udp.send_packet(37,packet)
    robot.update(upstream)
    ploter.update(*Dynamics.fk(robot))
    current = time.time()
    dt = current-start
      # pause 5.5 seconds
    print dt

#
# while(1):
#     packet = 15*[0]
#     upstream = udp.send_packet(37, packet)
#     robot.update(upstream)
#     ploter.update(*Dynamics.fk(robot))
#     pass
# packet[3] = robot.angle_to_encoder(0)
# packet[4] = robot.angle_to_encoder(0)
# packet[5] = 0
# upstream =  udp.send_packet(37,packet)
# print upstream
# robot.update(upstream)
# ploter.update(*Dynamics.fk(robot))
# print robot.q





