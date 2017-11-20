
import numpy as np
from math import pi
class Robot(object):
    # TODO add dosctrings to functions
    def __init__(self):
        self._inertia = [[1, 1, 1], [1, 1, 1], [1, 1, 1]] # Ix Iy Iz
        self._mass = [1, 1, 1]
        self._lengths = [1, 1, 1]
        self._centroid = [ 0.5,0.5,0.5]
        self.q = [0,0,0]
        self.qd = [ 0,0,0]
        self.qdd = [0, 0, 0]
        self.tau = [0,0,0]

    def update_q(self,q):
        self.q = q

    def update_qd(self,qd):
        self.qd = qd

    def update_qdd(self,qdd):
        self.qdd = qdd

    def update_tau(self, tau):
        self.tau = tau

    def update(self, state):
        # TODO figure out how to get qdd

        for i in xrange(3):
            self.q[i ] = self.encoders_to_angles(state[i * 3 + 0 + 1])
            self.qd[i] = self.encoders_to_angles(state[i * 3 + 2 + 1])
            self.tau[i] = state[i * 3 + 2 + 1]

        self.q[2] = self.q[2] - 0.5*pi

        pass
    def encoders_to_angles(self,ticks):
        # TODO create function to convert the tick to angles
        return ticks*(1/11.44)*(2*pi/360)
        pass

    # TODO figure out what to do with the load cells to get real data


    def unpack(self):
        return self._inertia, self._mass, self._lengths, self._centroid



