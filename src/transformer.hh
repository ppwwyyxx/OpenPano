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
		const std::vector<Feature>& f1, & f2;

		Matrix cal_transform(const std::vector<int>&) const;

		Matrix cal_homo_transform(const std::vector<int>&) const;
		Matrix cal_homo_transform2(const std::vector<int>&) const;

		Matrix cal_affine_transform(const std::vector<int>&) const;

		Matrix cal_rotate_homo_transform(const std::vector<int>&) const;

		int cal_inliers(const Matrix &) const;

		std::vector<int> get_inliers(const Matrix &) const;

	public:
		TransFormer(const MatchData& m_match, const std::vector<Feature>& m_f1, const std::vector<Feature>& m_f2):
			match(m_match), f1(m_f1), f2(m_f2) { }

		Matrix get_transform();

		static Vec2D cal_project(const Matrix &, const Vec2D&);

		static real_t get_focal_from_matrix(const Matrix& m);
};
