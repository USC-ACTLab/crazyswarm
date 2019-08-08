from mpl_toolkits.mplot3d import Axes3D  # NOQA
import matplotlib.pyplot as plt


class VisMatplotlib:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.ax.set_zlim([0, 3])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.plot = None
        self.timeAnnotation = self.ax.annotate("Time", xy=(0, 0), xycoords='axes fraction', fontsize=12, ha='right', va='bottom')

    def update(self, t, crazyflies):
        xs = []
        ys = []
        zs = []
        cs = []
        for cf in crazyflies:
            x, y, z = cf.position()
            color = cf.ledRGB
            xs.append(x)
            ys.append(y)
            zs.append(z)
            cs.append(color)

        if self.plot is None:
            self.plot = self.ax.scatter(xs, ys, zs, c=cs)
        else:
            self.plot._offsets3d = (xs, ys, zs)
            self.plot.set_facecolors(cs)
            self.plot.set_edgecolors(cs)
            self.plot._facecolor3d = self.plot.get_facecolor()
            self.plot._edgecolor3d = self.plot.get_edgecolor()

        self.timeAnnotation.set_text("{} s".format(t))
        plt.pause(0.0001)
