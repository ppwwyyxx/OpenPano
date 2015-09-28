// File: cylinder.hh
// Date: Tue Apr 30 23:49:42 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "lib/geometry.hh"
#include "lib/image.hh"
#include "feature.hh"

class Sphere {
	public:
		Vec center;
		int r;

		Sphere(int m_r, const Vec& m_center):
			center(m_center), r(m_r) { }

		inline real_t dist(const Vec& p) const
		{ return (center - p).mod(); }

		inline bool contain(const Vec& p) const
		{ return dist(p) < r; }

		Vec2D proj(const Vec& p) const;

		inline Vec2D proj(const Vec2D& p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec2D proj_r(const Vec2D&) const;
};

class Cylinder {
	public:
		Vec center;
		int r;

		Cylinder(int m_r, const Vec& m_center):
			center(m_center), r(m_r)
		{}

		Vec2D proj(const Vec& p) const;
		// return (angle with x) and (angle vertical)

		inline Vec2D proj(const Vec2D& p) const
		{ return proj(Vec(p.x, p.y, 0)); }

		Vec2D proj_r(const Vec2D& p) const;

};

class CylProject {
	public:
		const Cylinder cyl;
		int sizefactor;

		CylProject(int m_r, const Vec& m_center, int m_size):
			cyl(m_r, m_center), sizefactor(m_size){}

		imgptr project(const imgptrc& img, std::vector<Feature>& ft) const;
};

class SphProject {
	public:
		const Sphere sph;
		int sizefactor;

		SphProject(int m_r, const Vec& m_center, int m_size):
			sph(m_r, m_center), sizefactor(m_size){}

		/*
		 *imgptr project(const imgptrc& img) const;
		 */
};
