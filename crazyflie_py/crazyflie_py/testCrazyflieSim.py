import numpy as np
from numpy.testing import assert_almost_equal

from cfsim import CrazyflieSim, TimeHelper

def main():

	time = TimeHelper()
	cf_id = 42
	initial = np.array([1, 1, 0])
	cf = CrazyflieSim(id, initial, time)

	duration = 2.5
	takeoff_height = 1.0
	cf.takeoff(takeoff_height, duration)

	p0 = cf.position()
	assert_almost_equal(p0, initial, decimal=3)

	time.step(duration / 2)
	pHalf = cf.position()
	assert(pHalf[2] > initial[2])
	assert(pHalf[2] < takeoff_height)

	time.step(duration / 2)
	p1 = cf.position()
	assert_almost_equal(p1[2], takeoff_height, decimal=3)


if __name__ == "__main__":
	main()
