from UDP.UDP import UDP
import math
import Robot
import helper
import PlotArm
from Dynamics import Dynamics
import numpy as np
from Haptic_Controller import GravityCompensationController

import time

thanos = Robot.Robot("thanos",id=0)
hela = Robot.Robot("hela",id=1)
#ploter = PlotArm.PlotArm()

udp = UDP(9876)

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PID_CONFIG = 65
STATUS = 38
TORQUE_CONTROL = 39


pidConstants = [0.001, 0.0005, .01, 0.002, 0.00025, 0.01, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
udp.send_packet(thanos._id,PID_CONFIG, pidConstants)
#pidConstants =[.001, 0.0005, .01, 1000, 0.00025, .001, 0.002, 0.0004, 0.01, 0, 0, 0, 0, 0, 0];
udp.send_packet(hela._id,PID_CONFIG, pidConstants)


packet = 15 * [0.0]



#move the arm in a wave
while (1):
    # u = controller.getTorque(robot)


    upstream = udp.send_packet(thanos._id, STATUS, packet)
    thanos.update(upstream)



    msg = helper.make_packet(thanos.q, [0, 0, 0], [0, 0, 0])

    upstream = udp.send_packet(hela._id, PID_CONTROL, msg)
    hela.update(upstream)
    #ploter.update(*Dynamics.fk(hela))
    # print robot.q










