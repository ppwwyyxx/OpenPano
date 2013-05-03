// File: warper.hh
// Date: Fri May 03 18:05:16 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>

#include "config.hh"
#include "image.hh"
#include "feature.hh"
#include "cylinder.hh"

class Warper {
	public:
		const real_t h_factor;
		Warper(real_t m_hfactor):
			h_factor(m_hfactor) {}

		void warp(imgptr& img, std::vector<Feature>& ft) const {
			int r = std::max(img->w, img->h) / 2;
			Vec cen(img->w / 2, img->h / 2 * h_factor, r * 2);
			CylProject cyl(r, cen, r * OUTPUT_SIZE_FACTOR);
			img = cyl.project(img, ft);
		}

		void warp(imgptr& img) const {
			Feature f;
			f.real_coor = Vec2D(0, 0);
			std::vector<Feature> ff = {f};
			warp(img, ff);
		}
};
