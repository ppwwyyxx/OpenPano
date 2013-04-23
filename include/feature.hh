// File: feature.hh
// Date: Tue Apr 23 10:20:28 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "geometry.hh"
#include <cstring>

class Feature {
	public:
		Coor coor;
		int no, ns; // octave / scale id
		Vec2D real_coor;
		int sub_scale;		// to be more precise?

		real_t descriptor[DESC_LEN];
		real_t scale_factor;
		real_t dir;

		Feature(){}

		Feature(const Feature& r) {
			coor = r.coor;
			no = r.no;
			ns = r.ns;
			real_coor = r.real_coor;
			dir = r.dir;
			sub_scale = r.sub_scale;
			scale_factor = r.scale_factor;
			memcpy(descriptor, r.descriptor, DESC_LEN * sizeof(real_t));
		}

};
