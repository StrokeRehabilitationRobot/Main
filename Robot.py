
import numpy as np
from math import pi
import helper


class Robot(object):
    """
    Robot class to hold state and description of the robot
    """
    def __init__(self,arm="thanos",id=0):
        """

        :type id: arm id
        :param arm: which arm do you want
        :param id: id of the arm
        """
        if arm == "thanos":
            self.arm1()

        else:
            self.arm2()

        self._id = id

        self._inertia = [[0.006757, 0.0006036, 0.0015514],
                         [0.001745, 0.0005596, 0.00006455],
                         [0.00706657, 0.0006254, 0.0015708]
                         ]
        self._mass = [1.01992, 0.3519, 0.22772]
        self._lengths = [0.25107, 0.191, 0.37843]
        self._centroid = [0.10424, 0.14550, 0.203]
        self._convert_factor = ((1 / 11.44) * (2 * pi / 360)) # converts from tick->rads

        self.q = [0,0,0]
        self.qd = [ 0,0,0]
        self.qdd = [0, 0, 0]
        self.tau = [0,0,0]

    def arm1(self):
        """
        params for arm thanos
        :return:
        """
        self._torque_offset = [0.5646, 0.5084, 0.5128]
        self._max_tau = [0.6, 0.55, 0.55]
        self._min_tau = [.45, .40, .4]

    def arm2(self):

        self._torque_offset = [0.5646, 0.5084, 0.5128]
        self._max_tau = [0.6, 0.55, 0.55]
        self._min_tau = [.45, .40, .4]


    def update(self, state):
        # TODO figure out how to get qdd
        """
        updates the state of the robot
        :param state: the message from the board
        :return:
        """

        for i in xrange(3):
            self.q[i ] = helper.encoder_to_angle(state[i * 3 + 0 + 1])
            self.qd[i] = helper.encoder_to_angle(state[i * 3 + 2 + 1])
            self.tau[i] = self.filter_tau(self.interpolate_tau(state[i * 3 + 2 + 1],i),i)

        self.q[2] -= 0.5 * pi

    @property
    def unpack(self):
        """

        :return: all the phyical params of the robot
        """
        return self._inertia, self._mass, self._lengths, self._centroid

    def interpolate_tau(self,updated_tau,i):
        # TODO figure out scaling for torque for each link
        """

        :param updated_tau:
        :param i:
        :return:
        """
        return updated_tau#10*updated_tau - 10*self._torque_offset[i]

    def filter_tau(self,interpolated_tau,i):
        # TODO figure out analog way of interpring torque
        if interpolated_tau > self._max_tau[i]:
            return 1
        elif interpolated_tau < self._min_tau[i]:
            return -1
        else:
            return 0


