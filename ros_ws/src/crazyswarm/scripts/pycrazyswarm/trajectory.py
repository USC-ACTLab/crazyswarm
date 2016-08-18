import csv
import rospy

from crazyflie_driver.msg import QuadcopterTrajectoryPoly
import snap

# TODO: remove ros dependency and use numpy instead; convert in crazyflie.py to ros datatype

def polyval(poly, t):
    x = 0.0
    i = len(poly) - 1
    while i >= 0:
        x = x * t + poly[i]
        i = i - 1
    return x

def polyval_xyz(poly4d, t):
    x = polyval(poly4d.poly_x, t)
    y = polyval(poly4d.poly_y, t)
    z = polyval(poly4d.poly_z, t)
    return [x, y, z]

class Trajectory:
    def __init__(self):
        self.polygons = [] # array of type QuadcopterTrajectoryPoly

    def load(self, fileName):
        self.polygons = []
        with open(fileName) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            firstRow = True
            for row in reader:
                if firstRow:
                    firstRow = False
                    continue

                poly = QuadcopterTrajectoryPoly()
                poly.duration = rospy.Duration.from_sec(float(row[0]))
                for i in range(1, 9):
                    poly.poly_x.append(float(row[i]))
                for i in range(9, 17):
                    poly.poly_y.append(float(row[i]))
                for i in range(17, 25):
                    poly.poly_z.append(float(row[i]))
                for i in range(25, 33):
                    poly.poly_yaw.append(float(row[i]))
                self.polygons.append(poly)

    '''
    Waypoints must be provided in the form:
        [[ x0, dx0, d2x0, ... ]
         [ x1, dx1, d2x0, ... ]
         [ x2, dx2, d2x2, ... ]
         [ ...            ... ]]
    Omitted derivatives will be left free.
    '''
    def optimize_waypoints(self, x_waypts, y_waypts, z_waypts, yaw_waypts, duration):
        print("x:", x_waypts)
        print("y:", y_waypts)
        print("z:", z_waypts)
        print("yaw:", yaw_waypts)
        x   = snap.Trajectory1D(x_waypts)
        y   = snap.Trajectory1D(y_waypts)
        z   = snap.Trajectory1D(z_waypts, 3)
        yaw = snap.Trajectory1D(yaw_waypts, 3)

        npts = len(x_waypts)
        for wp in [y_waypts, z_waypts, yaw_waypts]:
            assert(len(wp) == npts)

        time_per_piece_guess = duration / float(npts - 1)

        path = snap.QrPath(x, y, z, yaw, time_per_piece_guess)
        # max radians - think it's not used unless we do whole-duration optimization
        path.tilt = 0.25
        T = path.optimize()
        assert(abs(sum(T) - duration) < 0.001)

        # TODO this should all be encapsulated within snap.py
        for p in [x, y, z, yaw]:
            p.cost(T)
            p.T = T
            p.p = p.p.reshape((-1, p.order + 1))

        for i in range(npts - 1):
            poly = QuadcopterTrajectoryPoly()
            poly.duration = rospy.Duration.from_sec(T[i])
            poly.poly_x = x.p[i,:]
            poly.poly_y = y.p[i,:]
            poly.poly_z = z.p[i,:]
            poly.poly_yaw = yaw.p[i,:]
            self.polygons.append(poly)

        # TEMP for plotting - should write function to evaluate "our" polynomials
        time = numpy.linspace(0, sum(T), 1000)
        self.plotdata = [[p(t) for t in time] for p in [x, y, z, yaw]]

    def stretch(self, timescale):
        # e.g. if s==2 the new polynomial will be stretched to take 2x longer
        recip = 1.0 / timescale
        for poly in self.polygons:
            scale = recip
            for i in range(0, 8):
                poly.poly_x[i] *= scale
                poly.poly_y[i] *= scale
                poly.poly_z[i] *= scale
                poly.poly_yaw[i] *= scale
                scale *= recip
            duration = poly.duration.to_sec()
            poly.duration = rospy.Duration.from_sec(duration * timescale)

    def scale(self, x, y, z, yaw):
        for poly in self.polygons:
            for i in range(0, 8):
                poly.poly_x[i] *= x
                poly.poly_y[i] *= y
                poly.poly_z[i] *= z
                poly.poly_yaw[i] *= yaw

    def shift(self, pos, yaw):
        for poly in self.polygons:
            poly.poly_x[0] += pos[0]
            poly.poly_y[0] += pos[1]
            poly.poly_z[0] += pos[2]
            poly.poly_yaw[0] += yaw

    def totalDuration(self):
        sum = 0.0
        for poly in self.polygons:
            sum += poly.duration.to_sec()
        return sum

    def evaluate(self, t):
        time = 0
        for polygon in self.polygons:
            if t < time + polygon.duration.to_sec():
                time_in_piece = t - time
                return polyval_xyz(polygon, time_in_piece)
            time += polygon.duration.to_sec()
        # if we get here, the trajectory has ended
        return polyval_xyz(self.polygons[-1], self.polygons[-1].duration.to_sec())
