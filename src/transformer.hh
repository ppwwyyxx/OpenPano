// File: transformer.hh
// Date: Fri May 03 04:50:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "matcher.hh"
#include "lib/matrix.hh"

class TransFormer {
	private:
		const MatchData& match;
		const std::vector<Descriptor>& f1, & f2;
		Matrix f2_homo_coor;	// 3xn

		Matrix calc_transform(const std::vector<int>&) const;

		Matrix calc_homo_transform(const std::vector<int>&) const;

		Matrix calc_affine_transform(const std::vector<int>&) const;

		Matrix calc_rotate_homo_transform(const std::vector<int>&) const;

		std::vector<int> get_inliers(const Matrix &) const;

	public:
		TransFormer(const MatchData& m_match, const std::vector<Descriptor>& m_f1, const std::vector<Descriptor>& m_f2):
			match(m_match), f1(m_f1), f2(m_f2),
			f2_homo_coor(3, match.size())
		{
			int n = match.size();
			REP(i, n) {
				Vec2D old = f2[match.data[i].y].coor;
				f2_homo_coor.at(0, i) = old.x;
				f2_homo_coor.at(1, i) = old.y;
			}
			REP(i, n) f2_homo_coor.at(2, i) = 1;
		}

		bool get_transform(Matrix* r);

		static Vec2D calc_project(const Matrix &, const Vec2D&);

		static real_t get_focal_from_matrix(const Matrix& m);
};
