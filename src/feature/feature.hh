// File: feature.hh
// Date: Fri May 03 03:36:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "lib/config.hh"
#include "lib/mat.h"
#include "lib/geometry.hh"
#include <cstring>

struct Descriptor {
	Vec2D coor;			// coordinate in range [0, w/h)
	std::vector<float> descriptor;

	// square of euclidean. use now_thres to early-stop
	float euclidean_sqr_base(const Descriptor& r, float now_thres) const {
		float ans = 0;
		REP(i, descriptor.size()) {
			ans += sqr(descriptor[i] - r.descriptor[i]);
			if (ans > now_thres)
				return -1;
		}
		return ans;
	}

	// sse version
#ifdef __SSE3__
	float euclidean_sqr_fast(const Descriptor& r, float now_thres) const;
#endif

	inline float euclidean_sqr(const Descriptor& r, float now_thres) const {
#ifdef __SSE3__
		return euclidean_sqr_fast(r, now_thres);
#else
		return euclidean_sqr_base(r, now_thres);
#endif
	}

};

std::vector<Descriptor> detect_SIFT(const Mat32f& img);

// A Scale-Space point
struct SSPoint {
	Coor coor;						// integer coordinate in the pyramid
	Vec2D real_coor;			// scaled coordinate
	int pyr_id, scale_id; // pyramid / scale id
	float dir;
	float scale_factor;
};
