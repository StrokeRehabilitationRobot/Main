from math import pi
import numpy as np


def encoder_to_angle(ticks):
    """

    :param ticks: encoder ticks
    :return: angle in rads
    """
    convert_factor = ((1 / 11.44) * (2 * pi / 360))  # converts from tick->rads
    return ticks * convert_factor

def angle_to_encoder(angle):
    """

    :param angle: angle of servo
    :return: encoder ticks(
    """
    convert_factor = ((1 / 11.44) * (2 * pi / 360))  # converts from tick->rads
    return angle / convert_factor

def make_packet(q,qd,tau):
    """

    :param q: joint values
    :param qd: joint vels values
    :param tau: torque vals
    :return:
    """
    packet = 15*[0.0]

    for i in xrange(3):
        packet[3*i] = angle_to_encoder(q[i])
        packet[3*i+1] = qd[i]
        packet[3*i+2] = tau[i]
    packet[6]+= angle_to_encoder(0.5*pi)
    return packet


def remap(u):
    """
    :param u: raw control
    :return: normilized control
    """
    tau = [0,0,0]
    newRange    = [ 0.0  , 2.50 ]
    j1_oldRange = [ 0.001, 0.97 ]
    j2_oldRange = [ 0.001, 0.45 ]
    tau[0] = u[0]#np.interp(u[0], [ -.1, .1   ], [ 0,2.5])
    tau[1] = -u[1]#np.interp(u[1], j1_oldRange, newRange)
    tau[2] = np.interp(u[2], j2_oldRange, newRange)
    return  tau