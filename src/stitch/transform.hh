//File: transform.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include "lib/geometry.hh"
#include "lib/matrix.hh"

class Homography : public Matrix {
	public:
		Homography():
			Matrix(3, 3)
		{}

		Vec trans(const Vec& m) const {
			const double* p = ptr();
			Vec ret(p[0] * m.x + p[1] * m.y + p[2] * m.z,
					p[3] * m.x + p[4] * m.y + p[5] * m.z,
					p[6] * m.x + p[7] * m.y + p[8] * m.z);
			ret.normalize();
			return ret;
		}

};

namespace projector {

typedef Vec2D (*homo2proj_t)(const Vec&);
typedef Vec (*proj2homo_t)(const Vec2D&);

namespace flat {

	static inline Vec2D homo2proj(const Vec &coord) {
		return Vec2D(coord.x / coord.z, coord.y / coord.z);
	}

	static inline Vec proj2homo(const Vec2D &coord) {
		return Vec(coord.x, coord.y, 1);
	}
}

namespace cylindrical {

	static inline Vec2D homo2proj(const Vec &coord) {
		return Vec2D(
				coord.x / hypotf(coord.y, coord.z),
				atan2(coord.y, coord.z));
	}

	static inline Vec proj2homo(const Vec2D &coord) {
		return Vec(coord.x, sin(coord.y), cos(coord.y));
	}
}

}
