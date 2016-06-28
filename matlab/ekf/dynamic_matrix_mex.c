#include "mex.h"
#include "math3d.h"
#include "ekf.c"

/*
inputs:
	q     4x1
	omega 3x1
	acc   3x1
	dt    1x1

outputs:
	F    15x15
*/

#ifdef MATLAB_MEX_FILE 
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 4) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nrhs", "4 inputs required.");
	}
	if (nlhs != 1) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nlhs", "1 outputs required.");
	}

	for (int i = 0; i < nrhs; ++i) {
		if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i])) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:notDouble", "All inputs must be type double.");
		}
	}

	struct quat q  = qload(mxGetPr(prhs[0]));
	struct vec ew  = vload(mxGetPr(prhs[1]));
	struct vec ea  = vload(mxGetPr(prhs[2]));
	double dt      = *mxGetPr(prhs[3]);

    plhs[0] = mxCreateDoubleMatrix(EKF_N, EKF_N, mxREAL);
	double *F  = mxGetPr(plhs[0]);

	float Ff[EKF_N][EKF_N];
	dynamic_matrix(q, ew, ea, dt, Ff);

	for (int i = 0; i < EKF_N; ++i) {
		for (int j = 0; j < EKF_N; ++j) {
			F[j * EKF_N + i] = Ff[i][j]; // row-major to column-major
		}
	}
}
#endif
