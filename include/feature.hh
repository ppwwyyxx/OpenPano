// File: feature.hh
// Date: Fri Apr 19 23:03:25 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "geometry.hh"
#include <cstring>
#define DESC_LEN 128

class Feature {
	public:
		Coor coor;
		int no, ns; // octave / scale id
		Coor real_coor;
		int sub_scale;		// to be more precise?

		real_t descriptor[DESC_LEN];
		real_t sig_octave;
		real_t dir;

		Feature(){}

		Feature(const Feature& r) {
			coor = r.coor;
			no = r.no;
			ns = r.ns;
			real_coor = r.real_coor;
			dir = r.dir;
			sub_scale = r.sub_scale;
			sig_octave = r.sig_octave;
			memcpy(descriptor, r.descriptor, DESC_LEN * sizeof(real_t));
		}
};
