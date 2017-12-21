from math import pi


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
    :return: encoder ticks
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