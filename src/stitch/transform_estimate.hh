// File: transform_estimate.hh
// Date: Fri May 03 04:50:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "lib/matrix.hh"
#include "lib/geometry.hh"
#include "match_info.hh"
#include "common/common.hh"

namespace pano {
class MatchData;
class Homography;
struct Descriptor;

// find transformation matrix between two set of matched feature
class TransformEstimation {
	public:
		// MatchData contains pairs of (f1_idx, f2_idx)
		// shape1 is (w, h) of the first image
		TransformEstimation(const MatchData& m_match,
				const std::vector<Vec2D>& kp1,
				const std::vector<Vec2D>& kp2,
				const Shape2D& shape1, const Shape2D& shape2);

		TransformEstimation(const TransformEstimation&) = delete;
		TransformEstimation& operator = (const TransformEstimation&) = delete;

		// get a transform matix from second(f2) -> first(f1)
		bool get_transform(MatchInfo* info);

		enum TransformType { Affine, Homo };

	private:
		const MatchData& match;
		const std::vector<Vec2D> &kp1, &kp2;
		const Shape2D shape1, shape2;

		float ransac_inlier_thres;
		TransformType transform_type;

		// homogeneous coordinate of points in image2
		Matrix f2_homo_coor;	// nx3

		// calculate best transform from given samples
		Homography calc_transform(const std::vector<int>&) const;

		// check if result can be further filtered,
		// fill in result to MatchInfo object,
		// and return whether it succeeds
		bool fill_inliers_to_matchinfo(
				const std::vector<int>&, MatchInfo*) const;

		// get inliers of a transform
		std::vector<int> get_inliers(const Homography&) const;
};
}
