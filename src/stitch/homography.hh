//File: homography.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include "lib/matrix.hh"
#include "lib/geometry.hh"
#include "common/common.hh"
namespace pano {

struct Shape2D;
class Homography;

// return a polygon in [-w/2,w/2] coor in image1
// homo: from 2 to 1
std::vector<Vec2D> overlap_region(
		const Shape2D& shape1, const Shape2D& shape2,
		const Matrix& homo, const Homography& inv);

class Homography {
	public:
		double data[9];

		Homography(){};

		Homography(const double (&arr)[9]) {
			REP(i, 9) data[i] = arr[i];
		}

		Homography(const Matrix& r) {
			m_assert(r.rows() == 3 && r.cols() == 3);
			const double* p = r.ptr();
			REP(i, 9) data[i] = p[i];
		}

		Homography transpose() const {
      return Homography{{
				data[0], data[3], data[6],
        data[1], data[4], data[7],
        data[2], data[5], data[8] }};
		}

		double& operator [] (int idx) { return data[idx]; }
		const double& operator [] (int idx) const { return data[idx]; }

		Homography operator * (const Homography& r) const;

		inline void operator += (const Homography& r) { REP(i, 9) data[i] += r.data[i]; }

		inline void mult(double r) { REP(i, 9) data[i] *= r; }

		// f[[ transform
		inline Vec trans(const Vec& m) const {
			Vec ret(data[0] * m.x + data[1] * m.y + data[2] * m.z,
					data[3] * m.x + data[4] * m.y + data[5] * m.z,
					data[6] * m.x + data[7] * m.y + data[8] * m.z);
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
			return trans_normalize(Vec(m.x, m.y, 1));
		}

		inline Vec2D trans2d(double x, double y) const {
			return trans2d(Vec2D(x, y));
		}
		// f]]

		inline void zero() { memset(data, 0, sizeof(data)); }

		// identity
		inline static Homography I() {
			Homography ret; ret.zero();
			REP(i, 3) ret.data[i * 3 + i] = 1;
			return ret;
		}

		void normalize() {
			double fac = 0;
			for (int i = 0; i < 9; i ++)
				fac += data[i] * data[i];
			fac = 9 / sqrt(fac);
			for (int i = 0; i < 9; i ++) {
				data[i] *= fac;
			}
		}

		Homography inverse(bool* succ= nullptr) const;

		//http://answers.opencv.org/question/2588/check-if-homography-is-good/
		//http://stackoverflow.com/questions/14954220/how-to-check-if-obtained-homography-matrix-is-good
		static bool health(const double* mat) {
			// perspective test
			if (fabs(mat[6]) > 2e-3)
				return false;
			if (fabs(mat[7]) > 2e-3)
				return false;
			// flip test
			Vec x0{mat[2], mat[5], mat[8]},	// trans(0,0)
					x1{mat[1]+mat[2],
						 mat[4]+mat[5],
						 mat[7]+mat[8]};	// trans(0,1)
			if (x1.y <= x0.y)
				return false;
			Vec x2{mat[0]+mat[1]+mat[2],
						 mat[3]+mat[4]+mat[5],
						 mat[6]+mat[7]+mat[8]};	// trans(1, 1);
			if (x2.x <= x1.x)
				return false;
			return true;
		}

		inline bool health() const {
			return Homography::health(data);
		}

		static Homography get_translation(double dx, double dy) {
      return Homography{{
				1, 0, dx,
        0, 1, dy,
        0, 0, 1 }};
		}

		Matrix to_matrix() const {
			Matrix ret(3, 3);
			REP(i, 9) ret.ptr()[i] = data[i];
			return ret;
		}

		friend std::ostream& operator << (std::ostream& os, const Homography& r) {
			os << "[" << r.data[0] << " " << r.data[1] << " " << r.data[2]
				 << "; " << r.data[3] << " " << r.data[4] << " " << r.data[5]
				 << "; " << r.data[6] << " " << r.data[7] << " " << r.data[8]
				 << "]";
			return os;
		}

		void serialize(std::ostream& os) const {
			REP(i, 8) os << data[i] << " ";
			os << data[8];
		}

		static Homography deserialize(std::istream& is) {
			Homography ret;
			REP(i, 9) is >> ret[i];
			return ret;
		}

};

}
