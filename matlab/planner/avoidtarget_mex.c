#include <mex.h>

#include "avoidtarget.c"
#include "pptraj.c"
#include "mexutil.h"


/*
inputs:
	time 1x1
	mouse 2x1

	call with only time to initialize

outputs:
	pos     2x49
*/

static struct avoid_target cfs[7][7];

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

static float const MAX_SPEED = 1.5;
static float const MAX_DISP = 1.2 * 0.5;
static float const MASS = 0.03;

#ifdef MATLAB_MEX_FILE 
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double t = *mxGetPr(prhs[0]);

	if (nrhs == 1) {
		for (int x = 0; x < 7; ++x) {
			for (int y = 0; y < 7; ++y) {
				struct vec home = mkvec(x * 0.5, y * 0.5, 0);
				//init_avoid_target(&cfs[x][y], home, MAX_SPEED, MAX_DISP, MASS, t);
				init_avoid_target(&cfs[x][y], home, MAX_SPEED, MAX_DISP, t);
			}
		}
	}
	else if (nrhs == 2) {
		double const *mouse = mxGetPr(prhs[1]);
		struct vec target = mkvec(mouse[0], mouse[1], 0);
		for (int x = 0; x < 7; ++x) {
			for (int y = 0; y < 7; ++y) {
				update_avoid_target(&cfs[x][y], target, t);
                eval_avoid_target(&cfs[x][y], t); // output not needed, just read pos from struct avoid_target
			}
		}
	}
	else {
        mexErrMsgIdAndTxt("Crazyswarm:AvoidTarget:nrhs", "1 or 2 inputs required.");
	}

	if (nlhs != 1) {
        mexErrMsgIdAndTxt("Crazyswarm:AvoidTarget:nlhs", "1 outputs required.");
	}

    plhs[0] = mxCreateDoubleMatrix(2, 49, mxREAL);
	double *positions = mxGetPr(plhs[0]);
	struct avoid_target const *cfs_1d = &cfs[0][0];
	for (int i = 0; i < 49; ++i) {
		//positions[2*i] = cfs_1d[i].last.pos.x;
		//positions[2*i+1] = cfs_1d[i].last.pos.y;
		positions[2*i] = cfs_1d[i].pos.x;
		positions[2*i+1] = cfs_1d[i].pos.y;
	}
}
#endif
