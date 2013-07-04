// File: cylinder.cc
// Date: Thu Jul 04 11:43:00 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "cylinder.hh"
using namespace std;

Vec2D Sphere::proj(const Vec& p) const {
	real_t phi = asin((p.y - center.y) / (dist(p)));
	real_t theta = asin((p.x - center.x) / hypot(p.x - center.x, p.z - center.z));
	return Vec2D(theta, phi);
}

Vec2D Sphere::proj_r(const Vec2D& p) const {
	real_t cosphi = cos(p.y);
	Vec dir(sin(p.x) * cosphi, sin(p.y), cos(p.x) * cosphi);
	dir.normalize();
	Vec dest = center + dir * (center.z / dir.z);		// ?
	return Vec2D(dest.x, dest.y);
}

Vec2D Cylinder::proj(const Vec& p) const {
	real_t longx = hypot(p.x - center.x, p.z - center.z);
	real_t theta = acos((center.x - p.x) / longx);
	return Vec2D(theta, (p.y - center.y) / longx);
}

Vec2D Cylinder::proj_r(const Vec2D& p) const {
	Vec dir(-cos(p.x), p.y, -sin(p.x));		// - means z axis point to photo
	dir.normalize();
	Vec dest = center - dir * (center.z / dir.z);
	return Vec2D(dest.x, dest.y);
}

imgptr CylProject::project(const imgptrc& img, vector<Feature>& ft) const {
	Vec2D min(numeric_limits<real_t>::max(), numeric_limits<real_t>::max()),
		  max(0, 0);
	REP(i, img->h) REP(j, img->w) {			// TODO finally: only use rect corners
		Vec2D newcoor = cyl.proj(Vec2D(j, i));
		min.update_min(newcoor), max.update_max(newcoor);
	}

	max = max * sizefactor, min = min * sizefactor;
	Vec2D realsize = max - min,
		  offset = min * (-1);
	Coor size = toCoor(realsize);
	real_t sizefactor_inv = 1.0 / sizefactor;

	imgptr ret = make_shared<Img>(size.x, size.y);
	ret->fill(Color::NO);
#pragma omp parallel for schedule(dynamic)
	REP(i, ret->h) REP(j, ret->w) {
		Vec2D oricoor = cyl.proj_r((Vec2D(j, i) - offset) * sizefactor_inv);
		if (between(oricoor.x, 0, img->w) && between(oricoor.y, 0, img->h))
			ret->set_pixel(i, j, img->get_pixel(oricoor));
	}

	for (auto & f : ft)
		f.real_coor = cyl.proj(f.real_coor) * sizefactor + offset;
	return move(ret);
}

/*
 *imgptr SphProject::project(const imgptrc& img) const {
 *    Vec2D min(numeric_limits<real_t>::max(), numeric_limits<real_t>::max()),
 *          max(0, 0);
 *    REP(i, img->h) REP(j, img->w) {			// TODO finally: only use rect corners
 *        Vec2D newcoor = sph.proj(Vec2D(j, i));
 *        min.update_min(newcoor), max.update_max(newcoor);
 *    }
 *
 *    max = max * sizefactor, min = min * sizefactor;
 *    Vec2D realsize = max - min,
 *          offset = min * (-1);
 *    Coor size = toCoor(realsize);
 *    real_t sizefactor_inv = 1.0 / sizefactor;
 *
 *    imgptr ret(new Img(size.x, size.y));
 *    ret->fill(Color::BLACK);
 *    REP(i, ret->h) REP(j, ret->w) {
 *        Vec2D oricoor = sph.proj_r((Vec2D(j, i) - offset) * sizefactor_inv);
 *        if (between(oricoor.x, 0, img->w) && between(oricoor.y, 0, img->h))
 *            ret->set_pixel(i, j, img->get_pixel(oricoor));
 *    }
 *    return move(ret);
 *}
 */
