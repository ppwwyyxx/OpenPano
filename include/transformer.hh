// File: transformer.hh
// Date: Sun Apr 21 16:36:49 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "matcher.hh"
#include "matrix.hh"

class TransFormer {
	private:
		const MatchData& match;

		Matrix compute_transform(const std::vector<std::pair<Coor, Coor>>&) const;

	public:
		TransFormer(const MatchData& m_match):
			match(m_match) { }

		Matrix get_transform();




};
