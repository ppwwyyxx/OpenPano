// File: warp.cc
// Date: Thu Jul 04 11:43:00 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "warp.hh"
#include "lib/imgproc.hh"
using namespace std;
using namespace feature;

namespace projector {

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

Mat32f CylinderProject::project(const Mat32f& img, vector<Descriptor>& ft) const {
	Vec2D min(numeric_limits<real_t>::max(), numeric_limits<real_t>::max()),
		  max(0, 0);
	REP(i, img.height()) REP(j, img.width()) {			// TODO finally: only use rect corners
		Vec2D newcoor = proj(Vec2D(j, i));
		min.update_min(newcoor), max.update_max(newcoor);
	}

	max = max * sizefactor, min = min * sizefactor;
	Vec2D realsize = max - min,
		  offset = min * (-1);
	Coor size = Coor(realsize.x, realsize.y);
	real_t sizefactor_inv = 1.0 / sizefactor;

	Mat32f mat(size.y, size.x, 3);
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

	for (auto & f : ft) {
		Vec2D coor(f.coor.x + img.width() / 2, f.coor.y + img.height() / 2);
		f.coor = proj(coor) * sizefactor + offset;
		f.coor.x -= mat.width() / 2;
		f.coor.y -= mat.height() / 2;
	}
	return mat;
}

void CylinderWarper::warp(Mat32f& mat, std::vector<Descriptor>& ft) const {
	int r = mat.width() * (config::FOCAL_LENGTH / 36.0);
	Vec cen(mat.width() / 2, mat.height() / 2 * h_factor, r);
	CylinderProject cyl(r, cen, r);
	mat = cyl.project(mat, ft);
}

}
