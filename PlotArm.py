import Dynamics
from Main.Robot import Robot

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.ion()
class PlotArm(object):

    def __init__(self):
        """
        create class variable for plot
        """
        # Set up plot

        plt.ylabel("y")
        plt.xlabel("x")
        self.figure = plt.figure()
        self.ax = self.figure.add_axes([0, 0, 1, 1], projection='3d')
        self.lines, = self.ax.plot([], [],[], '-o')
        # # Autoscale on unknown axis and known lims on the other


        # Other stuff
        self.ax.grid()


    def update(self, p_1,p_2,p_3):


                #self.lines.set_zdata(zdata)
        xdata = [p_1[0], p_2[0], p_3[0] ]
        ydata = [p_1[1], p_2[1], p_3[1]]
        zdata = [p_1[2], p_2[2], p_3[2]]

        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        self.lines.set_3d_properties(zdata)


        self.ax.relim()
        self.ax.autoscale_view()

        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


#
# xdata = []
# ydata = []
# #zdata = []
# ploter = PlotArm()
# for i in range(100):
#     import time
#     p_1 = (i, 0, 1,)
#     p_2 = (0, 1, 3,)
#     p_3 = (0, 2, 6,)
#     time.sleep(1)
#     ploter.update(p_1,p_2,p_3)




