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
