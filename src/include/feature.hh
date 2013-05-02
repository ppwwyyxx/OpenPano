// File: feature.hh
// Date: Fri May 03 03:36:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "config.hh"
#include "geometry.hh"
#include <cstring>

class Feature {
	public:
		Coor coor;
		Vec2D real_coor;
		int no, ns; // octave / scale id
		real_t dir;
		real_t scale_factor;

		real_t descriptor[DESC_LEN];

		Feature(){}

		Feature(const Feature& r):
			coor(r.coor), real_coor(r.real_coor), no(r.no), ns(r.ns), dir(r.dir),
			scale_factor(r.scale_factor){
			memcpy(descriptor, r.descriptor, DESC_LEN * sizeof(real_t));
		}

};
