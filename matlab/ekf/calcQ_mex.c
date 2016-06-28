#include "mex.h"
#include "ekf.c"
#include <string.h> // memcpy

/*
inputs:
	dt   1x1
	q    4x1
	ew   3x1
	ea   3x1
	na   3x1
	nba  3x1
	nw   3x1
	nbw  3x1

outputs:
	Q    15x15
*/

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 8) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nrhs", "8 inputs required.");
	}
	if (nlhs != 1) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nlhs", "1 outputs required.");
	}

	for (int i = 0; i < nrhs; ++i) {
		if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i])) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:notDouble", "All inputs must be type double.");
		}
	}

	double dt      = *mxGetPr(prhs[0]);
	struct quat q  = qload(mxGetPr(prhs[1]));
	struct vec ew  = vload(mxGetPr(prhs[2]));
	struct vec ea  = vload(mxGetPr(prhs[3]));
	struct vec na  = vload(mxGetPr(prhs[4]));
	struct vec nba = vload(mxGetPr(prhs[5]));
	struct vec nw  = vload(mxGetPr(prhs[6]));
	struct vec nbw = vload(mxGetPr(prhs[7]));

    plhs[0] = mxCreateDoubleMatrix(EKF_N, EKF_N, mxREAL);
	double *Q  = mxGetPr(plhs[0]);

	float Qf[EKF_N][EKF_N];
	ZEROARR(Qf);

	addQ(dt, q, ew, ea, Qf);
	for (int i = 0; i < EKF_N; ++i) {
		for (int j = 0; j < EKF_N; ++j) {
			Q[j * EKF_N + i] = Qf[i][j]; // row- to column-major
		}
	}
}
