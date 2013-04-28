// File: cylinder.hh
// Date: Sun Apr 28 19:52:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "geometry.hh"
#include "image.hh"
/*
 *
 *class Sphere {
 *    public:
 *        Vec center;
 *        real_t r;
 *
 *        Sphere(const Vec& m_center, real_t m_r):
 *            center(m_center), r(m_r) { }
 *
 *        virtual ~Sphere(){}
 *
 *        inline real_t dist(const Vec& p) const
 *        { return (center - p).mod(); }
 *
 *        inline bool contain(const Vec& p) const
 *        { return dist(p) < r; }
 *
 *        Vec proj(Vec p) const {
 *            m_assert(!contain(p));
 *            Vec real = (p - center).get_normalized();
 *            cout  << (p - center).mod() << endl;
 *            return real;
 *        }
 *
 *        Vec proj(Coor p) const
 *        { return proj(Vec(p.x, p.y, 0)); }
 *
 *        Vec proj_r(Vec dir) const {
 *            cout << dir << endl;
 *            real_t inter_dist = -(real_t)center.z / dir.z;
 *            cout  << inter_dist << endl;
 *            return center + dir * inter_dist;
 *        }
 *};
 */

class Cylinder {
	public:
		int r;
		Vec center;

		Cylinder(int m_r, const Vec& m_center) {
			r = m_r;
			center = m_center;
		}

		Vec2D proj(const Vec& p) const;
		// return (angle with x) and (angle vertical)

		Vec2D proj(const Vec2D& p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec2D proj_r(const Vec2D& p) const;

};

class CylProject {
	public:
		const Cylinder cyl;
		int sizefactor;

		CylProject(int m_r, const Vec& m_center, int m_size):
			cyl(m_r, m_center){
			sizefactor = m_size;
		}

		imgptr project(const imgptrc& img) const;

};
