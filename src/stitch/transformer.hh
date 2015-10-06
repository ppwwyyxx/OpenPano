// File: transformer.hh
// Date: Fri May 03 04:50:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "feature/matcher.hh"
#include "lib/matrix.hh"
#include "transform.hh"

// find transformation matrix between two set of matched feature
class TransFormer {
	private:
		const MatchData& match;
		const std::vector<Descriptor>& f1, & f2;

		// homogeneous coordinate of points in f2
		Matrix f2_homo_coor;	// 3xn

		Homography calc_transform(const std::vector<int>&) const;

		Homography calc_homo_transform(const std::vector<int>&) const;

		Homography calc_affine_transform(const std::vector<int>&) const;

		std::vector<int> get_inliers(const Homography &) const;

	public:
		TransFormer(const MatchData& m_match, const std::vector<Descriptor>& m_f1, const std::vector<Descriptor>& m_f2):
			match(m_match), f1(m_f1), f2(m_f2),
			f2_homo_coor(3, match.size())
		{
			int n = match.size();
			REP(i, n) {
				Vec2D old = f2[match.data[i].second].coor;
				f2_homo_coor.at(0, i) = old.x;
				f2_homo_coor.at(1, i) = old.y;
			}
			REP(i, n) f2_homo_coor.at(2, i) = 1;
		}

		bool get_transform(Homography* r);

		static real_t get_focal_from_matrix(const Matrix& m);
};
