// File: feature.hh
// Date: Fri May 03 03:36:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "lib/config.hh"
#include "lib/mat.h"
#include "lib/geometry.hh"
#include <cstring>

namespace feature {

struct Descriptor {
	Vec2D coor;			// coordinate in range [0, w/h)
	std::vector<float> descriptor;

	// square of euclidean. use now_thres to early-stop
	float euclidean_sqr(const Descriptor& r, float now_thres) const;

	int hamming(const Descriptor& r) const;
};

class FeatureDetector {
	public:
		virtual ~FeatureDetector() = default;

		std::vector<Descriptor> detect_feature(const Mat32f& img) const;
		virtual std::vector<Descriptor> do_detect_feature(const Mat32f& img) const = 0;
};

class SIFTDetector : public FeatureDetector {
	public:
		std::vector<Descriptor> do_detect_feature(const Mat32f& img) const;
};


class BriefPattern;
class BRIEFDetector : public FeatureDetector {
	public:
		BRIEFDetector();
		virtual ~BRIEFDetector();
		std::vector<Descriptor> do_detect_feature(const Mat32f& img) const;

		std::unique_ptr<BriefPattern> pattern;
};

// A Scale-Space point
struct SSPoint {
	Coor coor;						// integer coordinate in the pyramid
	Vec2D real_coor;			// scaled coordinate
	int pyr_id, scale_id; // pyramid / scale id
	float dir;
	float scale_factor;
};

}
