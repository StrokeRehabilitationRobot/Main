from UDP.UDP import UDP
import Robot
import PlotArm
from Dynamics import Dynamics
import time


robot = Robot.Robot()
ploter = PlotArm.PlotArm()

VEL_CONTROL = 48
VEL_TARGET = 42
PID_CONTROL = 37
PI_CONFIG = 65

udp = UDP(9876)



packet = 15*[0,0,0]

t0 = time.time()
upstream =  udp.send_packet(37,packet)
print time.time()-t0
while(1):
    t0 = time.time()
    upstream =  udp.send_packet(37,packet)
    print time.time()-t0
    print upstream
    robot.update(upstream)
    ploter.update(*Dynamics.fk(robot))
    print robot.q






