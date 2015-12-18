// File: color.hh
// Date: Sat May 04 12:50:06 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "geometry.hh"
#include <limits>

namespace pano {

class Color: public Vector<float> {
	public:
		constexpr explicit Color(float r = 0, float g = 0, float b = 0):
			Vector(r, g, b){}

		explicit Color(const Vector<float>& v):
			Vector<float>(v) {}

		explicit Color(const float* p) : Vector<float>(p) {}

		static constexpr float C_EPS = 1e-4;

		bool black() const
		{ return is_zero(C_EPS); }

		void check() const {
			// if (!between(x, 0, 2) || !between(y, 0, 2) || !between(z, 0, 2))
			// 	std::cout << *this << std::endl;
			m_assert(x >= 0 && x <= 1 + std::numeric_limits<double>::epsilon());
			m_assert(y >= 0 && y <= 1 + std::numeric_limits<double>::epsilon());
			m_assert(z >= 0 && z <= 1 + std::numeric_limits<double>::epsilon());
		}

		void normalize();

		Color operator * (float p) const
		{ return Color(x * p, y * p, z * p); }

		Color operator * (const Color& c) const
		{ return Color(x * c.x, y * c.y, z * c.z); }

		Color operator + (const Color &v) const
		{ return Color(x + v.x, y + v.y, z + v.z); }

		static const Color WHITE, BLACK, RED, BLUE, NO;
};


}
