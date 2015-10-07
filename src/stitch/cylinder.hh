// File: cylinder.hh
// Date: Tue Apr 30 23:49:42 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "lib/geometry.hh"
#include "lib/mat.h"
#include "feature/feature.hh"

class CylinderProject {
	public:
		Vec center;
		int r;
		int sizefactor;

		CylinderProject(int m_r, const Vec& m_center, int m_size):
			center(m_center), r(m_r),
			sizefactor(m_size){}

		Mat32f project(const Mat32f& img, std::vector<feature::Descriptor>& ft) const;

	private:
		// return (angle with x) and (angle vertical)
		Vec2D proj(const Vec& p) const;

		inline Vec2D proj(const Vec2D& p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec2D proj_r(const Vec2D& p) const;
};



class CylinderWarper {
	public:
		const real_t h_factor;
		CylinderWarper(real_t m_hfactor):
			h_factor(m_hfactor) {}

		void warp(Mat32f& mat, std::vector<feature::Descriptor>& ft) const;

		inline void warp(Mat32f& mat) const {
			feature::Descriptor f;
			f.coor = Vec2D(0, 0);
			std::vector<feature::Descriptor> ff = {f};
			warp(mat, {ff});
		}
};
