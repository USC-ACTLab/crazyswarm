#include <mex.h>

#include "ekf.c"
#include "mexutil.h"


/*
inputs:
	p       3x1
	v       3x1
	q       4x1
	omega   3x1
	acc     3x1
	dt      1x1

outputs:
	p       3x1
	v       3x1
	q       4x1
*/
#ifdef MATLAB_MEX_FILE 
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 6) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nrhs", "6 inputs required.");
	}
	if (nlhs != 3) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nlhs", "3 outputs required.");
	}

	for (int i = 0; i < nrhs; ++i) {
		if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i])) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:notDouble", "All inputs must be type double.");
		}
	}

	
	struct ekf old;
	ZEROARR(old.P);
	old.pos = vload(mxGetPr(prhs[0]));
	old.vel = vload(mxGetPr(prhs[1]));
	old.quat = qload(mxGetPr(prhs[2]));
	struct vec const wv = vload(mxGetPr(prhs[3]));
	struct vec const av = vload(mxGetPr(prhs[4]));
	float w[3]; vstoref(wv, w);
	float a[3]; vstoref(av, a);
	float const      dt = *mxGetPr(prhs[5]);

	struct ekf new;
	ekf_imu(&old, &new, a, w, dt);
	mxPrintVec(new.pos);
	mxPrintVec(new.vel);
	mxPrintVec(new.quat);

    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
	vstore(new.pos, mxGetPr(plhs[0]));

    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
	vstore(new.vel, mxGetPr(plhs[1]));

    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL);
	qstore(new.quat, mxGetPr(plhs[2]));
}
#endif
