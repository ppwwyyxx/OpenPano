//File: match_info.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>
#pragma once
#include <utility>
#include <vector>
#include <iostream>

#include "lib/geometry.hh"
#include "homography.hh"
#include "common/common.hh"

namespace pano {

struct MatchInfo {
	// coordinate is half-shifted
	typedef std::pair<Vec2D, Vec2D> PCC;			// to, from
	std::vector<PCC> match;
	float confidence = 0;		// negative value indicates number of inlier, for debug
	Homography homo;

	void reverse() {
		for (auto& c : match)
			std::swap(c.first, c.second);
	}

	void serialize(std::ostream& os) const {
		os << confidence << " ";
		homo.serialize(os);
		os << " " << match.size();
		for (auto& p : match) {
			os << " " << p.first.x << " "
				<< p.first.y << " " << p.second.x << " "
				<< p.second.y;
		}
	}

	static MatchInfo deserialize(std::istream& is) {
		MatchInfo ret;
		is >> ret.confidence;
		ret.homo = Homography::deserialize(is);
		int match_size;
		is >> match_size;

		ret.match.resize(match_size);
		REP(i, match_size) {
			PCC& p = ret.match[i];
			is >> p.first.x >> p.first.y >> p.second.x >> p.second.y;
		}
		return ret;
	}
};

struct Shape2D {
	int w, h;
	Shape2D(int w, int h): w(w), h(h) {}

	inline double halfw() const { return w * 0.5; }
	inline double halfh() const { return h * 0.5; }

	inline Vec2D center() const { return Vec2D{halfw(), halfh()}; }

	// 1 2
	// 3 4	return corner in [-w/2,w/2] coordinate
	inline std::vector<Vec2D> shifted_corner() const {
		return {Vec2D{-w*0.5, -h*0.5}, Vec2D{w*0.5,-h*0.5},
						Vec2D{-w*0.5, h*0.5}, Vec2D{w*0.5, h*0.5}};
	}

	// whether a point in shifted coor is inside the shape
	inline bool shifted_in(Vec2D p) const {
		return (p.x >= -w*0.5 && p.x < w*0.5 && p.y >= -h*0.5 && p.y < h*0.5);
	}

	friend std::ostream& operator << (std::ostream& os, const Shape2D& s) {
		os << "w=" << s.w << ",h=" << s.h;
		return os;
	}
};

}
