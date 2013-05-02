// File: warper.hh
// Date: Fri May 03 04:51:03 2013 +0800
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
			int r = max(img->w, img->h) / 2;
			Vec cen(img->w / 2, img->h / 2 * h_factor, r * 2);
			CylProject cyl(r, cen, r * OUTPUT_SIZE_FACTOR);
			img = cyl.project(img, ft);
		}
};
