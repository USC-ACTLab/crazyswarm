#include "mex.h"
#include <math.h>

#define CHOLSL_FLOAT double
#include "cholsl.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 1) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nrhs", "1 inputs required.");
	}
	if (nlhs != 1) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nlhs", "1 outputs required.");
	}

	for (int i = 0; i < nrhs; ++i) {
		if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i])) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:notDouble", "All inputs must be type double.");
		}
	}

	double *a = mxGetPr(prhs[0]);
	int const n = mxGetN(prhs[0]);
	if (mxGetM(prhs[0]) != n) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:cholinv", "argument must be square");
	}

    plhs[0] = mxCreateDoubleMatrix(n, n, mxREAL);
	double *ainv  = mxGetPr(plhs[0]);

	double *scratch = malloc(n * sizeof(double));
	cholsl(a, ainv, scratch, n);
	free(scratch);
}
