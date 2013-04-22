// File: transformer.hh
// Date: Mon Apr 22 18:42:20 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "matcher.hh"
#include "matrix.hh"

class TransFormer {
	private:
		const MatchData& match;

		Matrix cal_transform(const std::vector<std::pair<Coor, Coor>>&) const;

		int cal_inliers(const Matrix &) const;

		std::vector<std::pair<Coor, Coor>> get_inliers(const Matrix &) const;

		Vec2D cal_project(const Matrix &, const Coor&) const;

	public:
		TransFormer(const MatchData& m_match):
			match(m_match) { }

		Matrix get_transform();




};
