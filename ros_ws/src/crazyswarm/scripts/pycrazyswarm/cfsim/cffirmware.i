%module cffirmware
%{
#include "math3d.h"
#include "pptraj.h"
#include "planner.h"
#include "packetdef.h"
%}

%include "math3d.h"
%include "pptraj.h"
%include "planner.h"
%include "packetdef.h"

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
%}
