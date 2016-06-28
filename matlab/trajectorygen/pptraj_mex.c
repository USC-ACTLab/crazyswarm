#include <mex.h>
#include "pptraj.h"

mxArray *create3D(mwSize d1, mwSize d2, mwSize d3)
{
	mwSize dims[] = { d1, d2, d3 };
	return mxCreateNumericArray(3, dims, mxSINGLE_CLASS, mxREAL);
}

// inputs:  coefs, mass, t
// outputs: pos, vel, omega, yaw

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 3) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nrhs", "3 inputs required.");
	}
	if (nlhs != 4) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:nlhs", "4 outputs required.");
	}

	for (int i = 0; i < nrhs; ++i) {
		if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i])) {
			mexErrMsgIdAndTxt("Crazyswarm:EKF:notDouble", "All inputs must be type double.");
		}
	}
	if (mxGetM(prhs[2]) != 1) {
        mexErrMsgIdAndTxt("Crazyswarm:EKF:dimensions", "Input t must be 1xN.");
	}
	mwSize N = mxGetN(prhs[2]);


    double const *coeffs =  mxGetPr(prhs[0]);
    double const mass    = *mxGetPr(prhs[1]);
    double const *t      =  mxGetPr(prhs[2]);

    plhs[0] = mxCreateDoubleMatrix(3, N, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(3, N, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(3, N, mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1, N, mxREAL);

	double *pos   = mxGetPr(plhs[0]);
	double *vel   = mxGetPr(plhs[1]);
	double *omega = mxGetPr(plhs[2]);
	double *yaw   = mxGetPr(plhs[3]);

    struct poly4d p;
	float *as1d = &(p.p[0][0]);
	for (int i = 0; i < (4 * PP_SIZE); ++i) {
		as1d[i] = coeffs[i];
	}
    p.duration = t[N-1];

	for (int i = 0; i < N; ++i) {
		struct traj_eval ev = poly4d_eval(&p, t[i], mass);
		vstore(ev.pos, pos + 3*i);
		vstore(ev.vel, vel + 3*i);
		vstore(ev.omega, omega + 3*i);
		yaw[i] = ev.yaw;
	}
}
