// File: color.hh
// Date: Tue Apr 23 15:33:17 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "geometry.hh"

#include <cmath>

class Color: public Vector {
	public:
		Color(real_t r = 0, real_t g = 0, real_t b = 0):
			Vector(r, g, b){}

		static constexpr real_t C_EPS = 1e-4;

		bool black() const
		{ return is_zero(C_EPS); }

		void check() const {
			if (!between(x, 0, 2) || !between(y, 0, 2) || !between(z, 0, 2))
				std::cout << *this << std::endl;
			m_assert(x >= 0 && x <= 1 + SEPS);
			m_assert(y >= 0 && y <= 1 + SEPS);
			m_assert(z >= 0 && z <= 1 + SEPS);
		}

		void normalize();

		Color operator * (real_t p) const
		{ return Color(x * p, y * p, z * p); }

		Color operator * (const Color& c) const
		{ return Color(x * c.x, y * c.y, z * c.z); }

		static const Color WHITE, BLACK, RED, BLUE;

		Color operator + (const Color &v) const
		{ return Color(x + v.x, y + v.y, z + v.z); }
};

