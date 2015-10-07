//File: match_info.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <utility>
#include <vector>

#include "lib/geometry.hh"
#include "transform.hh"

struct MatchInfo {
	typedef std::pair<Vec2D, Vec2D> PCC;			// to, from
	std::vector<PCC> match;
	float confidence;
	Homography homo;

};
