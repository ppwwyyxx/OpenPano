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

namespace stitch {
class MatchInfo;

// find transformation matrix between two set of matched feature
class TransformEstimation {
	public:
		// MatchData contains pairs of (f1_idx, f2_idx)
		TransformEstimation(const feature::MatchData& m_match,
				const std::vector<feature::Descriptor>& m_f1,
				const std::vector<feature::Descriptor>& m_f2);

		// get a transform matix from second(f2) -> first(f1)
		bool get_transform(MatchInfo* info);

		enum TransformType { Affine, Homo };

	private:
		const feature::MatchData& match;
		const std::vector<feature::Descriptor> &f1, &f2;
		TransformType transform_type;

		// homogeneous coordinate of points in f2
		Matrix f2_homo_coor;	// 3xn

		// calculate best transform from given samples
		Matrix calc_transform(const std::vector<int>&) const;

		// fillin result to MatchInfo object
		void fill_inliers_to_matchinfo(
				const std::vector<int>&, MatchInfo*) const;

		// get inliers of a transform
		std::vector<int> get_inliers(const Matrix &) const;

		static const int ESTIMATE_MIN_NR_MATCH = 6;

};
}
