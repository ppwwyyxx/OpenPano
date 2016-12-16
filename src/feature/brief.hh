//File: brief.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include <utility>
#include "feature.hh"
#include "common/common.hh"
// BRIEF: Binary Robust Independent Elementary Features

namespace pano {

struct BriefPattern {
	int s;	// size
	std::vector<std::pair<int, int>> pattern;
};


// Brief algorithm implementation
class BRIEF {
	public:
		BRIEF(const Mat32f& img, const std::vector<SSPoint>&,
				const BriefPattern&);
		BRIEF(const BRIEF&) = delete;
		BRIEF& operator = (const BRIEF&) = delete;

		std::vector<Descriptor> get_descriptor() const;

		// s: size of patch. n: number of pair
		static BriefPattern gen_brief_pattern(int s, int n);

	protected:
		const Mat32f& img;
		const std::vector<SSPoint>& points;
		const BriefPattern& pattern;

		Descriptor calc_descriptor(const SSPoint&) const;
};

}
