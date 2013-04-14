// File: feature.hh
// Date: Sun Apr 14 23:18:50 2013 +0800
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
		int dir;
		int sub_scale;
		real_t descriptor[DESC_LEN];

		Feature(){}

		Feature(const Feature& r) {
			coor = r.coor;
			no = r.no;
			ns = r.ns;
			real_coor = r.real_coor;
			dir = r.dir;
			sub_scale = r.sub_scale;
			memcpy(descriptor, r.descriptor, DESC_LEN * sizeof(real_t));
		}
};
