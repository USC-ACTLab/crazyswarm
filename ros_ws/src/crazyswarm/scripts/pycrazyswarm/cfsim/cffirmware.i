%module cffirmware
%{
#include "math3d.h"
#include "pptraj.h"
#include "avoidtarget.h"
#include "planner.h"
#include "packetdef.h"
%}

%include "math3d.h"
%include "pptraj.h"
%include "avoidtarget.h"
%include "planner.h"
%include "packetdef.h"

%inline %{
void poly4d_set(struct poly4d *poly, int dim, int coef, float val)
{
    poly->p[dim][coef] = val;
}
void pp_set_piece(struct piecewise_traj *pp, int i, struct poly4d const *poly)
{
    pp->pieces[i] = *poly;
}
void plan_set_ppback(struct planner *p, struct piecewise_traj const *pp)
{
    *p->ppBack = *pp;
}
%}
