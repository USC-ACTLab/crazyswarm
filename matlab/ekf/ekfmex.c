#include "mex.h"
#include "ekf.c"
#include "mexutil.h"
#include <string.h> // memcpy

/*
inputs:
	t    1xN
	acc  3xN
	gyro 3xN
	pos  3xN
	quat 4xN

outputs:
	pos  3xN
	vel  3xN
	quat 4xN
	b_w  3xN b_a  3xN
*/

void ekf_imu_d(struct ekf const *ekf_prev, struct ekf *ekf, double const acc[3], double const gyro[3], float dt)
{
	float accf[3] = { acc[0], acc[1], acc[2] };
	float gyrof[3] = { gyro[0], gyro[1], gyro[2] };
	ekf_imu(ekf_prev, ekf, accf, gyrof, dt);
}

void ekf_vicon_d(struct ekf const *old, struct ekf *new, double const pos[3], double const quat[4])//, double *debug)
{
	float posf[] = { pos[0], pos[1], pos[2] };
	float quatf[] = { quat[0], quat[1], quat[2], quat[3] };
	ekf_vicon(old, new, posf, quatf);
}

mxArray *create3D(mwSize d1, mwSize d2, mwSize d3)
{
	mwSize dims[] = { d1, d2, d3 };
	return mxCreateNumericArray(3, dims, mxSINGLE_CLASS, mxREAL);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 6) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nrhs", "5 inputs required.");
	}
	if (nlhs != 6) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nlhs", "6 outputs required.");
	}

	for (int i = 0; i < nrhs; ++i) {
		if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i])) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:notDouble", "All inputs must be type double.");
		}
	}
	if (mxGetM(prhs[0]) != 1) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:dimensions", "Input t must be 1xN.");
	}
	mwSize N = mxGetN(prhs[0]);
	for (int i = 1; i < (nrhs - 1); ++i) { // last input is P
		if (mxGetN(prhs[i]) != N) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:dimensions", "All inputs must have same number of rows");
		}
	}
	// TODO verify input dimensions besides n-points
	
	// DEBUG
	//N = 100;

    double const *t    = mxGetPr(prhs[0]);
    double const *acc  = mxGetPr(prhs[1]);
    double const *gyro = mxGetPr(prhs[2]);
    double const *pos  = mxGetPr(prhs[3]);
    double const *quat = mxGetPr(prhs[4]);
    double const *P    = mxGetPr(prhs[5]);

    plhs[0] = mxCreateDoubleMatrix(3, N, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(3, N, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(4, N, mxREAL);
    plhs[3] = mxCreateDoubleMatrix(3, N, mxREAL);
    plhs[4] = mxCreateDoubleMatrix(3, N, mxREAL);
	plhs[5] = create3D(EKF_N, EKF_N, N);
	double *pos_out  = mxGetPr(plhs[0]);
	double *vel_out  = mxGetPr(plhs[1]);
	double *quat_out = mxGetPr(plhs[2]);
	double *b_w_out  = mxGetPr(plhs[3]);
	double *b_a_out  = mxGetPr(plhs[4]);
	float *P_out     = mxGetData(plhs[5]);
	if (P_out == NULL) {
		mexErrMsgIdAndTxt("Crazyswarm:EKF:allocation", "could not allocate covariance output");
	}

	float const dt = (t[1] - t[0]) / 1000; // TODO handle inconsistent dt? i.e. read array in loop?

	struct ekf a, b;
	struct ekf *front = &a, *back = &b;

	int const N_VEL_SMOOTH = 10;
	struct vec vel_init = vdiv(vsub(vload(pos + 3*N_VEL_SMOOTH), vload(pos)), N_VEL_SMOOTH * dt);

	float pos_init[3] = {pos[0], pos[1], pos[2]};
	float quat_init[4] = {quat[0], quat[1], quat[2], quat[3]};

	ekf_init(back, pos_init, vmem(&vel_init), quat_init);

	int const IMU_HZ = 500;
	int const VICON_TAKE_1_PER = 10; // vicon comes at approx 100 hz. skip 10 to get 10 hz.
	//int const VICON_HZ = IMU_HZ / VICON_SKIP;
	int vicon_tick = 0;
	int n_vicons = 0;

	int const P_dim = EKF_N * EKF_N;
	int const P_size = P_dim * sizeof(float);

	for (int i = 0; i < N; ++i) {

		ekf_imu_d(back, front, acc + 3*i, gyro + 3*i, dt);
		vstore(front->pos, pos_out + 3*i);
		vstore(front->vel, vel_out + 3*i);
		//vstore(front->acc_world, b_a_out + 3*i);
		qstore(front->quat, quat_out + 4*i);
		memcpy(P_out + P_dim*i, &front->P[0][0], P_size);

		// flip
		struct ekf *temp = front; front = back; back = temp;

		bool changed = (i > 0) && vneq(vload(pos + 3*i), vload(pos + 3*i - 3));
		if (changed) {
			if ((vicon_tick % VICON_TAKE_1_PER) == 0) {
				ekf_vicon_d(back, front, pos + 3*i, quat + 4*i);
				struct ekf *temp = front; front = back; back = temp;
				++n_vicons;
			}
			++vicon_tick;
		}
	}
	mexPrintf("n vicons C: %d\n", n_vicons);
}
