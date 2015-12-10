// File: transform_estimate.hh
// Date: Fri May 03 04:50:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "lib/matrix.hh"
#include "homography.hh"

namespace pano {
class MatchData;
struct Descriptor;
struct MatchInfo;
struct Shape2D;

// find transformation matrix between two set of matched feature
class TransformEstimation {
	public:
		// MatchData contains pairs of (f1_idx, f2_idx)
		// shape1 is (w, h) of the first image
		TransformEstimation(const MatchData& m_match,
				const std::vector<Descriptor>& m_f1,
				const std::vector<Descriptor>& m_f2,
				const Shape2D& shape1);

		// get a transform matix from second(f2) -> first(f1)
		bool get_transform(MatchInfo* info);

		enum TransformType { Affine, Homo };

	private:
		const MatchData& match;
		const std::vector<Descriptor> &f1, &f2;
		float ransac_inlier_thres;
		TransformType transform_type;

		// homogeneous coordinate of points in f2
		Matrix f2_homo_coor;	// 3xn

		// calculate best transform from given samples
		Matrix calc_transform(const std::vector<int>&) const;

		// fill in result to MatchInfo object
		void fill_inliers_to_matchinfo(
				const std::vector<int>&, MatchInfo*) const;

		// determine whether a set of inlier is geometrically good
		bool good_inlier_set(const std::vector<int>&) const;

		// get inliers of a transform
		std::vector<int> get_inliers(const Matrix &) const;
};
}
