// File: warp.hh
// Date: Tue Apr 30 23:49:42 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "lib/geometry.hh"
#include "lib/mat.h"
#include "feature/feature.hh"
#include "match_info.hh"
#include "common/common.hh"

namespace pano {

class CylinderProject {
	public:
		Vec center;
		int r;
		int sizefactor;

		CylinderProject(int m_r, const Vec& m_center, int m_size):
			center(m_center), r(m_r),
			sizefactor(m_size){}

		// project key points in an image, together with the image
		Mat32f project(const Mat32f& img, std::vector<Vec2D>& pts) const;

		// project key points in an image, given the image shape
		// return the projected image offset
		Vec2D project(Shape2D& shape, std::vector<Vec2D>& pts) const;

	private:
		// return (angle with x) and (angle vertical)
		Vec2D proj(const Vec& p) const;

		inline Vec2D proj(const Vec2D& p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec2D proj_r(const Vec2D& p) const;
};

class CylinderWarper {
	public:
		explicit CylinderWarper(real_t m_hfactor):
			h_factor(m_hfactor) {}

		// warp image together with key points
		void warp(Mat32f& mat, std::vector<Vec2D>& kpts) const {
			mat = get_projector(
					mat.width(), mat.height()).project(mat, kpts);
		}

		// warp keypoints given image shape
		void warp(Shape2D& shape, std::vector<Vec2D>& kpts) const {
			get_projector(shape.w, shape.h).project(shape, kpts);
		}

		// warp image only
		inline void warp(Mat32f& mat) const {
			std::vector<Vec2D> a;
			warp(mat, a);
		}

	protected:
		CylinderProject get_projector(int w, int h) const;
		const real_t h_factor;
};


}
