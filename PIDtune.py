from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import time
import csv

robot = Robot.Robot()
ploter = PlotArm.PlotArm()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65

udp = UDP(9876)
baseConstants = [0.001, 0.0002, 0.01];
shoulderConstants = [0.002, 0.00025, 0.01];
elbowConstants = [0.002, 0.0004, 0.01];





packet = 15*[0,0,0]
sinWaveInc = 10;
sin_range = 400;
loop = 100
count = 0
angles = []
pidConstants = [0.01, 0, 0, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0, ];
upstream = udp.send_packet(PID_CONFIG, pidConstants)
while(count<loop):
    count +=1

    # for k in xrange(sinWaveInc):
    #     incremtal = float(k / sinWaveInc);
    #
    #     for j in xrange(4):
    #         packet[(j * 3) + 0] = (math.sin(incremtal * math.pi * 2.0) * sin_range) + (sin_range);
    #         packet[(j * 3) + 1] = 0
    #         packet[(j * 3) + 2] = 3
    i = 0
    packet[0] = robot.angle_to_encoder(0.25*math.pi)
    packet[3] = robot.angle_to_encoder(0)
    packet[6] = robot.angle_to_encoder(0)

    upstream =  udp.send_packet(37,packet)
    robot.update(upstream)
    angles.append(robot.q[0])

    ploter.update(*Dynamics.fk(robot))
    print count



with open("angle01", 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
    wr.writerow(angles)



print "done"