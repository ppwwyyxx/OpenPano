// File: transform_estimate.hh
// Date: Fri May 03 04:50:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "lib/matrix.hh"
#include "transform.hh"

namespace feature {
	class MatchData;
	struct Descriptor;
}
class MatchInfo;

// find transformation matrix between two set of matched feature
class TransformEstimation {
	private:
		const feature::MatchData& match;
		const std::vector<feature::Descriptor>& f1, & f2;

		// homogeneous coordinate of points in f2
		Matrix f2_homo_coor;	// 3xn

		Homography calc_transform(const std::vector<int>&) const;

		void fill_inliers_to_matchinfo(const std::vector<int>&, MatchInfo*) const;

		std::vector<int> get_inliers(const Homography &) const;

	public:
		TransformEstimation(const feature::MatchData& m_match,
				const std::vector<feature::Descriptor>& m_f1,
				const std::vector<feature::Descriptor>& m_f2);

		bool get_transform(MatchInfo* info);
};
