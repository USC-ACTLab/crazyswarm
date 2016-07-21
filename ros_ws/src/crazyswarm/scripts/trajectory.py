import rospy
import csv
from crazyflie_driver.msg import QuadcopterTrajectoryPoly

# TODO: remove ros dependency and use numpy instead; convert in crazyflie.py to ros datatype

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

    def totalDuration(self):
        sum = 0.0
        for poly in self.polygons:
            sum += poly.duration.to_sec()
        return sum

