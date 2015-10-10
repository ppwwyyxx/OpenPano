//File: match_info.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <utility>
#include <vector>

#include "lib/geometry.hh"
#include "transform.hh"

struct MatchInfo {
	// coordinate is half-shifted
	typedef std::pair<Vec2D, Vec2D> PCC;			// to, from
	std::vector<PCC> match;
	float confidence = 0;
	Homography homo;

	void reverse() {
		for (auto& c : match) {
			auto p = c.first;
			c.first = c.second;
			c.second = p;
		}
	}
};
