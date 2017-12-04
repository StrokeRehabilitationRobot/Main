
import numpy as np
from math import pi
class Robot(object):
    # TODO add dosctrings to functions
    def __init__(self):
        self._inertia = [[0.006757, 0.0006036, 0.0015514],
                         [0.001745, 0.0005596, 0.00006455],
                         [0.00706657, 0.0006254, 0.0015708]
                        ]
        self._mass = [1.01992, 0.3519, 0.22772]
        self._lengths = [0.25107, 0.191, 0.37843]
        self._centroid =  [0.10424, 0.14550, 0.203]
        self.q = [0,0,0]
        self.qd = [ 0,0,0]
        self.qdd = [0, 0, 0]
        self.tau = [0,0,0]
        self._convert_factor = ((1/11.44)*(2*pi/360))
        self._torque_offset = [0.44420,0.5081,0.489]
        self.min_tau = [0,0,0]

    def update_q(self,q):
        self.q = q

    def update_qd(self,qd):
        self.qd = qd

    def update_qdd(self,qdd):
        self.qdd = qdd

    def update_tau(self, tau):
        tau = round(tau,1)

    def update(self, state):
        # TODO figure out how to get qdd

        for i in xrange(3):
            self.q[i ] = self.encoder_to_angle(state[i * 3 + 0 + 1])
            self.qd[i] = self.encoder_to_angle(state[i * 3 + 2 + 1])
            self.tau[i] = filter_tau(Interpolate_tau(state[i * 3 + 2 + 1],i),i)

        self.q[2] -= 0.5 * pi

        pass
    def encoder_to_angle(self,ticks):
        # TODO create function to convert the tick to angles
        return ticks * self._convert_factor
        pass

    # TODO figure out what to do with the load cells to get real data

    def angle_to_encoder(self,angle):
        # TODO create function to convert the tick to angles
        return angle / self._convert_factor
        pass


    def unpack(self):
        return self._inertia, self._mass, self._lengths, self._centroid

    def Interpolate_tau(self,updated_tau,i):
        return 10*updated_tau - self._torque_offset[i]

    def filter_tau(self,interpolated_tau,i):
        if interpolated_tau < 1.2*min_tau[i]:
            return 0
        else:
            return interpolated_tau




