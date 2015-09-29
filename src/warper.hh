// File: warper.hh
// Date: Fri May 03 18:05:16 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>

#include "lib/config.hh"
#include "lib/mat.h"
#include "feature.hh"
#include "cylinder.hh"

class Warper {
	public:
		const real_t h_factor;
		Warper(real_t m_hfactor):
			h_factor(m_hfactor) {}

		void warp(Mat32f& mat, std::vector<Feature>& ft) const {
			int r = std::max(mat.width(), mat.height()) / 2;
			Vec cen(mat.width() / 2, mat.height() / 2 * h_factor, r * 2);
			CylProject cyl(r, cen, r * OUTPUT_SIZE_FACTOR);
			mat = cyl.project(mat, ft);
		}

		void warp(Mat32f& mat) const {
			Feature f;
			f.real_coor = Vec2D(0, 0);
			std::vector<Feature> ff = {f};
			warp(mat, ff);
		}
};
