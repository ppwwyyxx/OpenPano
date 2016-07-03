// File: warp.cc
// Date: Thu Jul 04 11:43:00 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "warp.hh"
#include "lib/imgproc.hh"
using namespace std;
using namespace pano;

namespace pano {

Vec2D CylinderProject::proj(const Vec& p) const {
	real_t x = atan((p.x - center.x) / r);
	real_t y = (p.y - center.y) / (hypot(p.x - center.x, r));
	return Vec2D(x, y);
}

Vec2D CylinderProject::proj_r(const Vec2D& p) const {
	real_t x = r * tan(p.x) + center.x;
	real_t y = p.y * r / cos(p.x) + center.y;
	return Vec2D(x, y);
}

Mat32f CylinderProject::project(const Mat32f& img, vector<Vec2D>& pts) const {
	Shape2D shape{img.width(), img.height()};
	Vec2D offset = project(shape, pts);

	real_t sizefactor_inv = 1.0 / sizefactor;

	Mat32f mat(shape.h, shape.w, 3);
	fill(mat, Color::NO);
#pragma omp parallel for schedule(dynamic)
	REP(i, mat.height()) REP(j, mat.width()) {
		Vec2D oricoor = proj_r((Vec2D(j, i) - offset) * sizefactor_inv);
		if (between(oricoor.x, 0, img.width()) && between(oricoor.y, 0, img.height())) {
			Color c = interpolate(img, oricoor.y, oricoor.x);
			float* p = mat.ptr(i, j);
			p[0] = c.x, p[1] = c.y, p[2] = c.z;
		}
	}

	return mat;
}

Vec2D CylinderProject::project(Shape2D& shape, std::vector<Vec2D>& pts) const {
	Vec2D min(numeric_limits<real_t>::max(), numeric_limits<real_t>::max()),
				max(0, 0);
	REP(i, shape.h) REP(j, shape.w) {			// TODO finally: only use rect corners
		Vec2D newcoor = proj(Vec2D(j, i));
		min.update_min(newcoor), max.update_max(newcoor);
	}

	max = max * sizefactor, min = min * sizefactor;
	Vec2D realsize = max - min,
		    offset = min * (-1);
	Coor size = Coor(realsize.x, realsize.y);

	for (auto & f : pts) {
		Vec2D coor(f.x + shape.w / 2, f.y + shape.h / 2);
		f = proj(coor) * sizefactor + offset;
		f.x -= size.x / 2;
		f.y -= size.y / 2;
	}
	shape.w = size.x, shape.h = size.y;
	return offset;
}


CylinderProject CylinderWarper::get_projector(int w, int h) const {
	// 43.266 = hypot(36, 24)
	int r = hypot(w, h) * (config::FOCAL_LENGTH / 43.266);
	Vec cen(w / 2, h / 2 * h_factor, r);
	return CylinderProject(r, cen, r);
}

}
