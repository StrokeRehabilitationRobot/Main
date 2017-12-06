from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import time


robot = Robot.Robot("arm1")
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
count = 0
while(1):
    count +=1

    pidConstants = [0.001, 0.0005, .01, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0, ];
    upstream = udp.send_packet(PID_CONFIG, pidConstants)
    # for k in xrange(sinWaveInc):
    #     incremtal = float(k / sinWaveInc);
    #
    #     for j in xrange(4):
    #         packet[(j * 3) + 0] = (math.sin(incremtal * math.pi * 2.0) * sin_range) + (sin_range);
    #         packet[(j * 3) + 1] = 0
    #         packet[(j * 3) + 2] = 3
    i = 0
    packet[0] = robot.angle_to_encoder(0)
    packet[3] = robot.angle_to_encoder(0)
    packet[6] = robot.angle_to_encoder(0)



    upstream =  udp.send_packet(37,packet)
    robot.update(upstream)
    print robot.tau

    ploter.update(*Dynamics.fk(robot))
    #print robot.q






