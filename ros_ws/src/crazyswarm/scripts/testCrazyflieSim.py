import numpy as np
from numpy.testing import assert_almost_equal

from cfsim import CrazyflieSim

def main():

	cf_id = 42
	initial = np.array([1, 1, 0])
	cf = CrazyflieSim(id, initial)

	t0 = 0
	t1 = 2.5
	takeoff_height = 1.0
	cf.takeoff(takeoff_height, t1 - t0, t0)

	p0 = cf.position(t0 + 0.001)
	assert_almost_equal(p0, initial, decimal=3)
	return

	pHalf = cf.position((t0 + t1) / 2)
	assert(pHalf[2] > initial[2])
	assert(pHalf[2] < takeoff_height)

	p1 = cf.position(t1)
	assert_almost_equal(p1[2], takeoff_height, decimal=3)


if __name__ == "__main__":
	main()
