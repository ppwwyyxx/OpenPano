// File: sphere.hh
// Date: Sat Apr 27 01:52:17 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "geometry.hh"

class Sphere {
	public:
		Vec center;
		real_t r;

		Sphere(const Vec& m_center, real_t m_r):
			center(m_center), r(m_r) { }

		virtual ~Sphere(){}

		inline real_t dist(const Vec& p) const
		{ return (center - p).mod(); }

		inline bool contain(const Vec& p) const
		{ return dist(p) < r; }

		Vec proj(Vec p) const {
			m_assert(!contain(p));
			Vec real = (p - center).get_normalized();
			cout  << (p - center).mod() << endl;
			return real;
		}

		Vec proj(Coor p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec proj_r(Vec dir) const {
			cout << dir << endl;
			real_t inter_dist = -(real_t)center.z / dir.z;
			cout  << inter_dist << endl;
			return center + dir * inter_dist;
		}
};

class Cylinder {
	public:
		int r;
		Vec center;
		Cylinder(int m_r, Vec m_center) {
			r = m_r;
			center = m_center;
		}

		Vec2D proj(const Vec& p) const {		// return (angle with x) and height
			real_t longx = hypot(p.x - center.x, p.z - center.z);
			real_t newy = r / longx * (p.y - center.y) + center.y;
			Vec2D xzpl(p.x - center.x, p.z - center.z);
			xzpl.normalize();
			real_t theta = acos(xzpl.dot(Vec2D(1, 0)));
			if ((abs(newy - center.y) > abs(p.y - center.y))) {
				cout << "proj newy is wrong" << endl;
				cout << p << endl;
				cout << center << endl;
				cout << longx << endl;
				cout << newy << endl;
			}
			return Vec2D(newy, theta);
		}

		Vec2D proj(const Vec2D& p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec2D proj_r(const Vec2D& p) const {
			Vec dir(r * cos(p.y), p.x - center.y, -r * sin(p.y));		// - means z axis point to photo
			dir.normalize();
			Vec dest = center + dir * (-center.z / dir.z);
			return Vec2D(dest.x, dest.y);
		}

};
