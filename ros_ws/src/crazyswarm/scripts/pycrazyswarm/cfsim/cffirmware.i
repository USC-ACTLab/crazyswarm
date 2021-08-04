%module cffirmware

// ignore GNU specific compiler attributes
#define __attribute__(x)

%{
#define SWIG_FILE_WITH_INIT
#include "collision_avoidance.h"
#include "math3d.h"
#include "pptraj.h"
#include "planner.h"
#include "stabilizer_types.h"
%}

%include "collision_avoidance.h"
%include "math3d.h"
%include "pptraj.h"
%include "planner.h"
%include "stabilizer_types.h"

%include "numpy.i"


%init %{
    import_array()
%}

%apply (int DIM1, float* IN_ARRAY1) {(int nOthers, float const *otherPositions)}

%inline %{
void poly4d_set(struct poly4d *poly, int dim, int coef, float val)
{
    poly->p[dim][coef] = val;
}
float poly4d_get(struct poly4d *poly, int dim, int coef)
{
    return poly->p[dim][coef];
}
struct poly4d* pp_get_piece(struct piecewise_traj *pp, int i)
{
    return &pp->pieces[i];
}
struct poly4d* malloc_poly4d(int size)
{
    return (struct poly4d*)malloc(sizeof(struct poly4d) * size);
}

struct vec vec2svec(struct vec3_s v)
{
  return mkvec(v.x, v.y, v.z);
}

struct vec3_s svec2vec(struct vec v)
{
  struct vec3_s vv = { .x = v.x, .y = v.y, .z = v.z, };
  return vv;
}

void collisionAvoidanceUpdateSetpointWrap(
  collision_avoidance_params_t const *params,
  collision_avoidance_state_t *collisionState,
  int nOthers,
  float const *otherPositions,
  setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state)
{
    nOthers /= 3;
    float *workspace = malloc(sizeof(float) * 7 * (nOthers + 6));
    collisionAvoidanceUpdateSetpointCore(
        params,
        collisionState,
        nOthers,
        otherPositions,
        workspace,
        setpoint, sensorData, state
    );
    free(workspace);
}
%}


%pythoncode %{
import numpy as np
%}

#define COPY_CTOR(structname) \
structname(struct structname const *x) { \
    struct structname *y = malloc(sizeof(struct structname)); \
    *y = *x; \
    return y; \
} \
~structname() { \
    free($self); \
} \

%extend vec {
    COPY_CTOR(vec)

    %pythoncode %{
        def __repr__(self):
            return "({}, {}, {})".format(self.x, self.y, self.z)

        def __array__(self):
            return np.array([self.x, self.y, self.z])

        def __len__(self):
            return 3

        def __getitem__(self, i):
            if 0 <= i and i < 3:
                return _cffirmware.vindex(self, i)
            else:
                raise IndexError("vec index must be in {0, 1, 2}.")

        # Unary operator overloads.
        def __neg__(self):
            return _cffirmware.vneg(self)

        # Vector-scalar binary operator overloads.
        def __rmul__(self, s):
            return _cffirmware.vscl(s, self)

        def __div__(self, s):
            return self.__truediv__(s)

        def __truediv__(self, s):
            return _cffirmware.vdiv(self, s)

        # Vector-vector binary operator overloads.
        def __add__(self, other):
            return _cffirmware.vadd(self, other)

        def __sub__(self, other):
            return _cffirmware.vsub(self, other)
    %}
};

%extend traj_eval {
    COPY_CTOR(traj_eval)
};
