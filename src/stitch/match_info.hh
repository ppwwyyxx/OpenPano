//File: match_info.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>
#pragma once
#include <utility>
#include <vector>

#include "lib/geometry.hh"
#include "transform.hh"

namespace stitch {

struct MatchInfo {
	// coordinate is half-shifted
	typedef std::pair<Vec2D, Vec2D> PCC;			// to, from
	std::vector<PCC> match;
	float confidence = 0;		// negative value indicates number of inlier, for debug
	Homography homo;

	void reverse() {
		for (auto& c : match) {
			auto p = c.first;
			c.first = c.second;
			c.second = p;
		}
	}
};

struct Shape2D {
	int w, h;
	Shape2D(int w, int h): w(w), h(h) {}

	inline double halfw() const { return w * 0.5; }
	inline double halfh() const { return h * 0.5; }
};

}
