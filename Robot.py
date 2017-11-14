

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
        # TODO create function to parse numpy array
        pass
    def encoders_to_angles(self):
        # TODO create function to convert the tick to angles
        pass

    # TODO figure out what to do with the load cells to get real data


    def unpack(self):
        return self._inertia, self._mass, self._lengths, self._centroid



