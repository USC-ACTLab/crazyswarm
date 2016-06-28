#include <mex.h>

#include "ekf.c"
#include "mexutil.h"


/*
inputs:
	p       3x1
	v       3x1
	q       4x1
	P       15x15
	omega   3x1
	acc     3x1
	dt      1x1

outputs:
	p       3x1
	v       3x1
	q       4x1
	P       15x15
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
	if (nlhs != 4) {
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
	d2f(P, AS_1D(old.P), EKF_N*EKF_N);

	struct vec const wv = vload(mxGetPr(prhs[4]));
	struct vec const av = vload(mxGetPr(prhs[5]));
	float w[3]; vstoref(wv, w);
	float a[3]; vstoref(av, a);
	float const      dt = *mxGetPr(prhs[6]);

	struct ekf new;
	ekf_imu(&old, &new, a, w, dt);

    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
	vstore(new.pos, mxGetPr(plhs[0]));

    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
	vstore(new.vel, mxGetPr(plhs[1]));

    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL);
	qstore(new.quat, mxGetPr(plhs[2]));

    plhs[3] = mxCreateDoubleMatrix(EKF_N, EKF_N, mxREAL);
	f2d(AS_1D(new.P), mxGetPr(plhs[3]), EKF_N*EKF_N);
}
#endif
