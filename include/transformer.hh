// File: transformer.hh
// Date: Tue Apr 23 18:34:49 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "matcher.hh"
#include "matrix.hh"

class TransFormer {
	private:
		const MatchData& match;

		Matrix cal_transform(const std::vector<std::pair<Vec2D, Vec2D>>&) const;

		Matrix cal_homo_transform(const std::vector<std::pair<Vec2D, Vec2D>>&) const;

		Matrix cal_affine_transform(const std::vector<std::pair<Vec2D, Vec2D>>&) const;

		int cal_inliers(const Matrix &) const;

		std::vector<std::pair<Vec2D, Vec2D>> get_inliers(const Matrix &) const;


	public:
		TransFormer(const MatchData& m_match):
			match(m_match) { }

		Matrix get_transform();

		static Vec2D cal_project(const Matrix &, const Vec2D&);
};
