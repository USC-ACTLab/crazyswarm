from __future__ import print_function

'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr

	Numerical method for minimizing the snap of a piecewise-
	polynomial flight trajectory (as seen in Trajectory1D):
	Charles Richter, Adam Bry, and Nicholas Roy
	http://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
'''

from scipy.optimize import minimize_scalar
from scipy.optimize import minimize
from scipy.misc import factorial
from time import time
import numpy as np

class QrPath:
	def __init__(self, x, y, z, psi, power=10.00, tilt=None, guess=5.00):
		# flat output space trajectories
		self.x = x
		self.y = y
		self.z = z
		self.psi = psi

		# limits
		self.power = power				# thrust [N]
		self.tilt = tilt				# theta/phi [rad]

		# polynomial length guess [s]
		self.guess = guess

		# number of waypoints
		self.n_wp = x.n_wp

		# physical constants
		self.m = 0.53		# total mass [kg]
		self.g = 9.81		# acceleration of gravity [m/s^2]
		self.Ix = 6.228e-3	# longitudinal inertia [kgm^2]
		self.Iy = 6.228e-3	# lateral inertia [kgm^2]

		# peak values [u1, u2, u3, theta, phi]
		self.peaks = [0, 0, 0, 0, 0]

	def __call__(self, t):
		r = [[], [], [], []]
		r[0] = [self.x(t, d=i) for i in range(5)]
		r[1] = [self.y(t, d=i) for i in range(5)]
		r[2] = [self.z(t, d=i) for i in range(5)]
		r[3] = [self.psi(t, d=i) for i in range(5)]
		return r

	def optimize(self):
		start_time = time()

		# Optimize the polynomial segment time ratio Tr.
		Tr0 = np.ones(self.n_wp - 1) * 1e3
		Tr_res = minimize(self.total_cost, Tr0, method='Nelder-Mead', options={'disp':False, 'maxiter':20})
		self.Tr = np.array([i / sum(Tr_res.x) for i in Tr_res.x])

		# Optimize the total trajectory time k.
		#k0 = (self.guess * (self.n_wp - 1),)
		#rho0 = self.guess
		#cons = ({'type': 'ineq', 'fun': lambda k: self.power - self.u1_peak(k[0])},
			#{'type': 'ineq', 'fun': lambda k: self.tilt - self.theta_peak(k[0])},
			#{'type': 'ineq', 'fun': lambda k: self.tilt - self.phi_peak(k[0])})
		#k_res = minimize(lambda k: k[0], k0, method='COBYLA', constraints=cons, options={'disp':True, 'maxiter':20, 'rhobeg':rho0})
		#self.k = k_res.x[0]
		#T = self.Tr * self.k
		self.k = self.guess * (self.n_wp - 1)
		T = self.Tr * self.k

		# Compute psi piecewise polynomial (ignored in optimization).
		self.psi.cost(T)
		self.psi.T = T
		self.psi.p = self.psi.p.reshape((-1, self.psi.order + 1))

		# Display results.
		print('\nT =', T)
		print('u1 peak:', round(self.peaks[0], 5), 'N')
		print('theta peak:', round(self.peaks[3] / np.pi, 5), 'pi rad')
		print('phi peak:', round(self.peaks[4] / np.pi, 5), 'pi rad')
		print('Computation time:', round(time() - start_time, 2), 's', end='\n\n')

		return T

	# Cost function for calculating Tr.
	def total_cost(self, T):
		return self.x.cost(T) + self.y.cost(T) + self.z.cost(T)

	def u1(self, t):
		return self.m * (self.z(t, d=2) + self.g)

	def u2(self, t):
		xd2 = self.x(t, d=2)
		xd3 = self.x(t, d=3)
		xd4 = self.x(t, d=4)
		zd2 = self.z(t, d=2)
		zd3 = self.z(t, d=3)
		zd4 = self.z(t, d=4)
		return 	self.Ix / (self.g + zd2) * ( xd4 - 		\
				2 * zd3 * ((xd3 * (zd2 + self.g) - 		\
				xd2 * zd3) / (zd2 + self.g)**2)) - 		\
				((xd2 * zd4) / (zd2 + self.g))

	def u3(self, t):	
		yd2 = self.y(t, d=2)
		yd3 = self.y(t, d=3)
		yd4 = self.y(t, d=4)
		zd2 = self.z(t, d=2)
		zd3 = self.z(t, d=3)
		zd4 = self.z(t, d=4)
		return	self.Iy / (self.g + zd2) * (-yd4 - 		\
				2 * zd3 * ((yd3 * (zd2 + self.g) - 		\
				yd2 * zd3) / (zd2 + self.g)**2)) - 		\
				((yd2 * zd4) / (zd2 + self.g))

	def theta(self, t):
		return  self.x(t, d=2) / (self.z(t, d=2) + self.g)

	def phi(self, t):
		return -self.y(t, d=2) / (self.z(t, d=2) + self.g)

	def u1_peak(self, k):
		T = k * self.Tr
		self.z.cost(T)
		self.z.T = T
		self.z.p = self.z.p.reshape((-1, self.z.order + 1))
		bnds = self.get_bounds(self.u1, k)
		u1_res = minimize_scalar(lambda t: -self.u1(t), bounds=bnds, method='bounded')
		self.peaks[0] = np.abs(self.u1(u1_res.x))
		return self.peaks[0]

	def theta_peak(self, k):
		T = k * self.Tr
		self.x.cost(T)
		self.x.T = T
		self.x.p = self.x.p.reshape((-1, self.x.order + 1))
		bnds = self.get_bounds(lambda t: np.abs(self.theta(t)), k)
		theta_res = minimize_scalar(lambda t: -np.abs(self.theta(t)), bounds=bnds, method='bounded')
		self.peaks[3] = np.abs(self.theta(theta_res.x))
		return self.peaks[3]

	def phi_peak(self, k):
		T = k * self.Tr
		self.y.cost(T)
		self.y.T = T
		self.y.p = self.y.p.reshape((-1, self.y.order + 1))
		bnds = self.get_bounds(lambda t: np.abs(self.phi(t)), k)
		phi_res = minimize_scalar(lambda t: -np.abs(self.phi(t)), bounds=bnds, method='bounded')
		self.peaks[4] = np.abs(self.phi(phi_res.x))
		return self.peaks[4]		

	# Returns a set of bounds for the minimizer to use. Necessary for accurate 
	# minimization of non-convex functions such as u1, u2, u3, and u4. This is 
	# a BRUTE FORCE method. We take rezo samples per piecewise polynomial
	# segment, find the time which results in the maximum, and return the two
	# time samples adjacent to that time.
	def get_bounds(self, fn, k, rezo=20):
		t_vals = np.linspace(0, k, rezo * (self.n_wp - 1))
		x_vals = [fn(t) for t in t_vals]
		m = max(x_vals)
		peak_time = t_vals[[i for i, j in enumerate(x_vals) if j == m][0]]
		bnds_inter = k / rezo / (self.n_wp - 1)
		if bnds_inter <= peak_time:	return (peak_time - bnds_inter, peak_time + bnds_inter)
		else: 						return (0, peak_time + bnds_inter)

class Trajectory1D:
	def __init__(self, wp, der=4):
		'''
		Waypoints must be provided in the form:
			[[ x0, dx0, d2x0, ... ]
			 [ x1, dx1, d2x0, ... ]
			 [ x2, dx2, d2x2, ... ]
			 [ ...            ... ]]
		Omitted derivatives will be left free, and any 
		derivatives up to and including the objective 
		(der) derivative will be made continous on the 
		waypoints.
		'''

		self.wp = np.array(wp)			# waypoints
		self.match = der				# derivative to minimize and match

		if len(wp) < 3: print('More waypoints required.')
		elif der < 2: 	print('Higher derivative required.')
		else:		self.init_QP()

	def __call__(self, t, d=0):
		for m in range(len(self.T)):
			if t > self.T[m] and m != len(self.T) - 1:
				t -= self.T[m]
			else: break
		P = 0
		for n in range(d, self.order + 1):
			P += self.p[m][n] * (factorial(n) / factorial(n - d)) * t**(n - d)
		return P

	def init_QP(self):
		# Some constants.
		self.order = self.match * 2 + 1		# polynomial order
		self.n_coefs = self.order + 1		# number of polynomial coefficients
		self.n_wp = np.shape(self.wp)[0]	# number of waypoints

		############### Q COMPUTATION ###############
		# Step 1: Derivate the polynomial to the objective degree.
		ps = np.ones(self.n_coefs)			# array of polynomial coefficients
		ts = np.arange(self.n_coefs)		# array of powers of t
		for d in range(self.match):
			for n in range(self.n_coefs):
				ps[n] *= n - d
				if ts[n] > 0: ts[n] -= 1

		# Step 2: Square the resulting polynomial. (Arrays become 2D.)
		Qp_tile = np.tile(ps, (self.n_coefs, 1))
		Qp_tile *= Qp_tile.T
		Qt_tile = np.tile(ts, (self.n_coefs, 1))
		Qt_tile += Qt_tile.T

		# Step 3: Integrate from 0 to T.
		Qp_tile *= 1 / (Qt_tile + 1)
		Qt_tile += 1

		# Step 4: Tile Ps and Ts to create Qp and Qt.
		z = np.zeros_like(Qp_tile)
		self.Qp = Qp_tile
		self.Qt = Qt_tile
		for i in range(self.n_wp - 2):
			z_h	= np.tile(z, (1, i + 1))
			z_v	= np.tile(z, (i + 1, 1))
			self.Qp	= np.hstack((self.Qp, z_v))
			self.Qp	= np.vstack((self.Qp, np.hstack((z_h, Qp_tile))))
			self.Qt	= np.hstack((self.Qt, z_v))
			self.Qt	= np.vstack((self.Qt, np.hstack((z_h, Qt_tile))))

		############### d COMPUTATION ###############
		d = [[]]
		for i in range(self.n_wp):
			# input values: specified waypoint derivatives
			d[0].extend(self.wp[i])

			# None: unspecificed waypoint derivatives
			if len(self.wp[i]) < (self.match + 1):
				d[0].extend([None] * ((self.match + 1) - len(self.wp[i])))

			# zeros: continuity of derivatives on waypoints
			if i != 0 and i != (self.n_wp - 1):
				d[0].extend(np.zeros(self.match + 1))
		self.d = np.array(d).T

		############### A COMPUTATION ###############
		# Matrix of polynomial coefficient multipliers.
		self.Ap_tile = [np.ones(self.n_coefs)]
		for i in range(self.match):
			p_temp = [self.Ap_tile[-1][n] * (n - i) for n in range(self.n_coefs)]
			self.Ap_tile = np.vstack((self.Ap_tile, p_temp))	

		# Matrix of powers of t.
		self.At_tile = [np.arange(self.n_coefs)]
		for i in range(self.match):
			t_temp = [j for j in self.At_tile[-1]]
			for n in range(self.n_coefs):
				if t_temp[n] > 0: t_temp[n] -= 1
			self.At_tile = np.vstack((self.At_tile, t_temp))

		# Zeros tile.
		self.Az_tile = np.zeros_like(self.Ap_tile)

		############### C COMPUTATION ###############
		# As we rearrange d, we build the permutation matrix.
		C1 = []				# permutation matrix (top)
		C2 = []				# permutation matrix (bottom)
		dF = [[]]			# fixed/specified derivatives

		for i in range(len(self.d)):
			if self.d[i][0] is not None:
				C1.append(np.zeros(len(self.d)))
				C1[-1][i] = 1
				dF[0].append(self.d[i][0])
			else:
				C2.append(np.zeros(len(self.d)))
				C2[-1][i] = 1
		if C2:
			self.C = np.vstack((C1, C2))
			self.dF = np.array(dF).T
			self.simple = False
		else: self.simple = True

	def cost(self, T):
		############### Q COMPUTATION ###############
		# Step 5: Q = Qp * T^Qt
		T_ = []
		for t in T: T_.extend([t for i in range(self.n_coefs)])
		self.Q = self.Qp * T_**self.Qt

		############### A COMPUTATION ###############
		# Tiling algorithm. A_tile = Ap_tile * t^.At_tile.
		# Row 1: First curve at t=0.
		self.A = self.Ap_tile * 0**self.At_tile
		self.A = np.hstack((self.A, self.Az_tile))
		# Row 2: First curve at t=T.
		second_row = self.Ap_tile * T[0]**self.At_tile
		third_row = second_row
		second_row = np.hstack((second_row, self.Az_tile))
		self.A = np.vstack((self.A, second_row))

		# Row 3: Derivative matching at second waypoint.
		# Here, we say that second_curve(t=0) - first_curve(t=T) = 0.
		third_row = np.hstack((third_row, -1.0 * self.Ap_tile * 0**self.At_tile))
		self.A = np.vstack((self.A, third_row))
		# Row 4: Second curve at t=T.
		fourth_row = np.hstack((self.Az_tile, self.Ap_tile * T[1]**self.At_tile))
		self.A = np.vstack((self.A, fourth_row))

		# Rows 5+
		for i in range(1, self.n_wp - 2):
			self.A = np.hstack((self.A, np.tile(self.Az_tile, (2 + 2 * i, 1))))
			first_row = np.tile(self.Az_tile, (1, i))
			first_row = np.hstack((first_row, self.Ap_tile * T[i]**self.At_tile, -1.0 * self.Ap_tile * 0**self.At_tile))
			second_row = np.tile(self.Az_tile, (1, i + 1))
			second_row = np.hstack((second_row, self.Ap_tile * T[i + 1]**self.At_tile))
			self.A = np.vstack((self.A, first_row, second_row))
		
		if self.simple == True:
			# No unknowns in d. Solve for p.
			self.p = np.dot(np.linalg.inv(self.A), self.d)

		else:
			############### R COMPUTATION ###############
			# R = CA^(-T)QA^(-1)C^(T)
			R = np.dot(self.C, np.linalg.inv(self.A).T)
			R = np.dot(R, self.Q)
			R = np.dot(R, np.linalg.inv(self.A))
			R = np.dot(R, self.C.T)

			########### OPTIMAL dP COMPUTATION ##########
			RFP = []
			RPP = []
			for i in range(len(R)):
				if i < len(self.dF):
					RFP.append(R[i][len(self.dF):])
				else:
					RPP.append(R[i][len(self.dF):])
			RFP = np.array(RFP)
			RPP = np.array(RPP)

			# dP* = -RPP^(-1)RFP^(T)dF
			self.dP = -np.linalg.inv(RPP)
			self.dP = np.dot(self.dP, RFP.T)
			self.dP = np.dot(self.dP, self.dF)

			################ p COMPUTATION ##############
			d_remake = np.vstack((self.dF, self.dP))
			d_remake = np.dot(self.C.T, d_remake)
			self.p = np.dot(np.linalg.inv(self.A), d_remake)

		############# RETURN TOTAL COST #############
		J = np.dot(self.p.T, self.Q)
		J = np.dot(J, self.p)[0][0]
		return J

