// File: transformer.hh
// Date: Fri May 03 04:50:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
//#include "feature/matcher.hh"
#include "lib/matrix.hh"
#include "transform.hh"

namespace feature {
	class MatchData;
	struct Descriptor;
}

// find transformation matrix between two set of matched feature
class TransFormer {
	private:
		const feature::MatchData& match;
		const std::vector<feature::Descriptor>& f1, & f2;

		// homogeneous coordinate of points in f2
		Matrix f2_homo_coor;	// 3xn

		Homography calc_transform(const std::vector<int>&) const;

		Homography calc_homo_transform(const std::vector<int>&) const;

		Homography calc_affine_transform(const std::vector<int>&) const;

		std::vector<int> get_inliers(const Homography &) const;

	public:
		TransFormer(const feature::MatchData& m_match,
				const std::vector<feature::Descriptor>& m_f1,
				const std::vector<feature::Descriptor>& m_f2);

		bool get_transform(Homography* r);

		static real_t get_focal_from_matrix(const Matrix& m);
};
