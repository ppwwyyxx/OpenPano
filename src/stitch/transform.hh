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

		Homography(const Matrix& r):
			Matrix(r) {}

		inline Vec trans(const Vec& m) const {
			const double* p = ptr();
			Vec ret(p[0] * m.x + p[1] * m.y + p[2] * m.z,
					p[3] * m.x + p[4] * m.y + p[5] * m.z,
					p[6] * m.x + p[7] * m.y + p[8] * m.z);
			ret.normalize();
			return ret;
		}

		inline Vec trans(const Vec2D& m) const {
			return trans(Vec(m.x, m.y, 1));
		}

		inline Vec2D trans2d(const Vec2D& m) const {
			Vec v = trans(Vec(m.x, m.y, 1));
			double denom = 1.0 / v.z;
			return Vec2D(v.x * denom, v.y * denom);
		}

		inline Vec2D trans2d(double x, double y) const {
			return trans2d(Vec2D(x, y));
		}

		// identity
		inline static Homography I() {
			Homography ret;
			ret.zero();
			double* p = ret.ptr();
			REP(i, 3) p[i * 3 + i] = 1;
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
