from __future__ import print_function
from math import *

FT_TO_M = 0.3048

# parameters
max_view_dist_ft = 30
max_view_dist = FT_TO_M * max_view_dist_ft
marker_diam = 6.0 / 1000.0

v16_fov_vert = radians(54.22)
v16_res_vert = 4096

v8_fov_vert = radians(47.15)
v8_res_vert = 2432

v5_fov_vert = radians(40.47)
v5_res_vert = 2048

def marker_diam_pixels(fov, res, name):
	viewsize = 2 * max_view_dist * tan(fov / 2.0)
	pixelsize = viewsize / res
	diam = marker_diam / pixelsize
	print("{0} marker diameter in pixels at {1} feet: {2}".format(
		name, max_view_dist_ft, diam))

marker_diam_pixels(v5_fov_vert, v5_res_vert, "V5");
marker_diam_pixels(v8_fov_vert, v8_res_vert, "V8");
marker_diam_pixels(v16_fov_vert, v16_res_vert, "V16");
