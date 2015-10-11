//File: transform.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include "lib/matrix.hh"
#include "lib/geometry.hh"

class Homography : public Matrix {
	public:
		Homography():
			Matrix(3, 3) {}

		Homography(const Matrix& r): Matrix(r) {}

		inline Vec trans(const Vec& m) const {
			const double* p = ptr();
			Vec ret(p[0] * m.x + p[1] * m.y + p[2] * m.z,
					p[3] * m.x + p[4] * m.y + p[5] * m.z,
					p[6] * m.x + p[7] * m.y + p[8] * m.z);
			return ret;
		}

		inline Vec2D trans_normalize(const Vec& m) const {
			Vec ret = trans(m);
			double denom = 1.0 / ret.z;
			return Vec2D(ret.x * denom, ret.y * denom);
		}

		inline Vec trans(const Vec2D& m) const {
			return trans(Vec(m.x, m.y, 1));
		}

		inline Vec trans(double x, double y) const {
			return trans(Vec(x, y, 1));
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

		void normalize() {
			double* mat = m_data.get();
			double fac = 0;
			for (int i = 0; i < 9; i ++)
				fac += mat[i] * mat[i];
			fac = 9 / sqrt(fac);
			for (int i = 0; i < 9; i ++) {
				mat[i] *= fac;
			}
		}

		inline Homography inverse(bool* succ= nullptr) const {
			// TODO rewrite it by calling eigen directly might speed up
			Homography ret;
			bool ok = Matrix::inverse(ret);
			if (succ==nullptr)
				m_assert(ok);
			else
				*succ = ok;
			return ret;
		}

		bool health() const {
			const double* mat = ptr();
			if (mat[8] < EPS)
				return false;
			// perspective test
			if (fabs(mat[6]) > 2e-3)
				return false;
			if (fabs(mat[7]) > 2e-3)
				return false;
			// flip test
			Vec x0 = trans(0, 0),
					x1 = trans(0, 1);
			if (x1.y <= x0.y)
				return false;
			Vec x2 = trans(1, 1);
			if (x2.x <= x1.x)
				return false;
			return true;
		}
};
