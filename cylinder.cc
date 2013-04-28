// File: cylinder.cc
// Date: Sun Apr 28 20:22:40 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "cylinder.hh"
using namespace std;

Vec2D Cylinder::proj(const Vec& p) const {
	real_t dy = p.y - center.y;
	Vec2D xzpl(p.x - center.x, p.z - center.z);
	real_t longx = xzpl.mod();
	xzpl.normalize();
	real_t theta = acos(xzpl.dot(Vec2D(-1, 0)));
	return Vec2D(theta, dy / longx);
}

Vec2D Cylinder::proj_r(const Vec2D& p) const {
	Vec dir(-r * cos(p.x), r * p.y, -r * sin(p.x));		// - means z axis point to photo
	dir.normalize();
	Vec dest = center + dir * (-center.z / dir.z);
	return Vec2D(dest.x, dest.y);
}

imgptr CylProject::project(const imgptrc& img) const {
	Vec2D min(numeric_limits<real_t>::max(), numeric_limits<real_t>::max()),
		  max(0, 0);
	REP(i, img->h) REP(j, img->w) {
		Vec2D newcoor = cyl.proj(Vec2D(j, i));
		min.update_min(newcoor), max.update_max(newcoor);
	}

	Vec2D realsize = max - min,
		  offset = min * (-sizefactor);
	Coor size = toCoor(realsize * sizefactor);

	imgptr ret(new Img(size.x, size.y));
	ret->fill(Color::BLACK);
	REP(i, ret->h) REP(j, ret->w) {
		Vec2D oricoor = cyl.proj_r((Vec2D(j, i) - offset) * (1.0 / sizefactor));
		if (between(oricoor.x, 0, img->w) && between(oricoor.y, 0, img->h)) {
			ret->set_pixel(i, j, img->get_pixel(oricoor.y, oricoor.x));
		}
	}
	return move(ret);
}
