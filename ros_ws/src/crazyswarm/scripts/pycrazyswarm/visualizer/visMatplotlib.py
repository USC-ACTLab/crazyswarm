from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class VisMatplotlib:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-5,5])
        self.ax.set_ylim([-5,5])
        self.ax.set_zlim([0,3])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.plot = None
        self.timeAnnotation = self.ax.annotate("Time", xy=(0, 0), xycoords='axes fraction', fontsize=12, ha='right', va='bottom')

    def update(self, t, crazyflies):
        xs = []
        ys = []
        zs = []
        for cf in crazyflies:
            x, y, z = cf.position()
            xs.append(x)
            ys.append(y)
            zs.append(z)

        if self.plot is None:
            self.plot = self.ax.scatter(xs, ys, zs)
        else:
            self.plot._offsets3d = (xs, ys, zs)

        self.timeAnnotation.set_text("{} s".format(t))
        plt.pause(0.0001)
