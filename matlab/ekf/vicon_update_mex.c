#include <mex.h>

#include "mexutil.h"
#include "ekf.c"


/*
inputs:
	p       3x1
	v       3x1
	q       4x1
	P       15x15
	R       6x6
	p_vicon 3x1
	q_vicon 4x1

outputs:
	p       3x1
	v       3x1
	q       4x1
	P       15x15
	debug   
*/

static inline void d2f(double const *d, float *f, int n)
{
	for (int i = 0; i < n; ++i) {
		f[i] = d[i];
	}
}

static inline void f2d(float const *f, double *d, int n)
{
	for (int i = 0; i < n; ++i) {
		d[i] = f[i];
	}
}


#ifdef MATLAB_MEX_FILE 
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 7) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nrhs", "7 inputs required.");
	}
	if (nlhs != 5) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nlhs", "4 outputs required.");
	}

	for (int i = 0; i < nrhs; ++i) {
		if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i])) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:notDouble", "All inputs must be type double.");
		}
	}

	struct ekf old;
	old.pos = vload(mxGetPr(prhs[0]));
	old.vel = vload(mxGetPr(prhs[1]));
	old.quat = qload(mxGetPr(prhs[2]));

	double const *P = mxGetPr(prhs[3]);
	d2f(P, AS_1D(old.P), EKF_N * EKF_N);

	//double const *R = mxGetPr(prhs[4]);
	//d2f(R, AS_1D(old.R), EKF_M * EKF_M);

	//struct vec  p_vicon  = vload(mxGetPr(prhs[5]));
	//struct quat q_vicon  = qload(mxGetPr(prhs[6]));

	double const *pd = mxGetPr(prhs[5]);
	double const *qd = mxGetPr(prhs[6]);
	float const p_vicon[3] = { pd[0], pd[1], pd[2] };
	float const q_vicon[4] = { qd[0], qd[1], qd[2], qd[3] };


	plhs[4] = mxCreateDoubleMatrix(EKF_N, EKF_M, mxREAL);

	struct ekf new;
	//vicon_update(&old, &new, p_vicon, q_vicon, mxGetPr(plhs[4]));
	ekf_vicon(&old, &new, p_vicon, q_vicon);


    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
	vstore(new.pos, mxGetPr(plhs[0]));

    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
	vstore(new.vel, mxGetPr(plhs[1]));

    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL);
	qstore(new.quat, mxGetPr(plhs[2]));

    plhs[3] = mxCreateDoubleMatrix(EKF_N, EKF_N, mxREAL);
	f2d(AS_1D(new.P), mxGetPr(plhs[3]), EKF_N * EKF_N);
}
#endif
