
from UDP.UDP import UDP
import math
import Robot
import PlotArm
from Dynamics import Dynamics
import numpy as np
from Haptic_Controller import GravityCompensationController
import helper
import time
from dmp_experiments.Python import DMP_runner




def way_points(path):
    movement = []
    count = 0
    while count < len(path)-2:

        p0 = path[count]
        p1 = path[count + 1]
        p2 = path[count + 2]

        m1 = None
        m2 = None
        dir = None

        # Get the two slopes
        if p1[0] == p0[0]:
            m1 = (p1[1]-p0[1])*float("inf")
        else:
            m1 = (p1[1] - p0[1])/(p1[0] - p0[0])

        if p2[0] == p1[0]:
            m2 = (p2[1] - p1[1]) * float("inf")
        else:
            m2 = (p2[1] - p1[1]) / (p2[0] - p1[0])


        # compare the slopes
        if m1 == float("inf") and m2 == float("inf"):
            dir = "N"
        elif m1 == -float("inf") and m2 == -float("inf"):
            dir = "S"
        elif m1 == 0 and m2 == 0:
            if p2[0] > p1[0]:
                dir = "W"
            else:
                dir = "E"
        elif m1 == 0 and m2 == float("inf"):
            if p2[0] > p0[0]:
                dir = "WN"
            else:
                dir = "EN"
        elif m1 == 0 and m2 == -float("inf"):
            if p2[0] > p0[0]:
                dir = "WS"
            else:
                dir = "ES"
        elif m2 == 0 and m1 == float("inf"):
            if p2[0] > p0[0]:
                dir = "NW"
            else:
                dir = "NE"
        elif m2 == 0 and m1 == -float("inf"):
            if p2[0] > p0[0]:
                dir = "SW"
            else:
                dir = "SE"


        movement.append(( dir, p0, p2 ))
        count += 2


    return movement


robot = Robot.Robot("arm1",id=1)
ploter = PlotArm.PlotArm()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65

udp = UDP(9876)


packet = 15*[0.0]

pidConstants = [0.001, 0.0005, .01, .001, .0005, .01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];

path = []
tau = 1
dt = 2.5*10**(-3)

way_points = way_points(path)

for via in way_points:

    file_x = via[0] + "_x.csv"
    file_y = via[0] + "_y.csv"
    file_z = via[0] + "_z.csv"

    runner_x = DMP_runner.step(file_x,via[1][0],via[2][0])
    runner_y = DMP_runner.step(file_y, via[1][1], via[2][1])
    runner_z = DMP_runner.step(file_z, 0, 0)

    for i in np.arange(0, int(tau / dt) + 1):


        runner_x.step(tau, dt)
        runner_y.step(tau, dt)
        runner_z.step(tau, dt)
        x = runner_x.y
        y = runner_y.y
        z = runner_z.y

        q1,q2,q3 =  Dynamics.ik(robot, (x,y,z))
        packet[0] = helper.angle_to_encoder(q1)
        packet[3] = helper.angle_to_encoder(q2)
        packet[6] = helper.angle_to_encoder(q3)


        upstream =  udp.send_packet(0,PID_CONTROL,packet)
        robot.update(upstream)
        ploter.update(*Dynamics.fk(robot))
    #print robot.q

