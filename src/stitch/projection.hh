//File: projection.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once

#include "lib/geometry.hh"
#include "common/common.hh"

namespace pano {

	typedef Vec2D (*homo2proj_t)(const Vec&);
	typedef Vec (*proj2homo_t)(const Vec2D&);

	namespace flat {
		static inline Vec2D homo2proj(const Vec &homo) {
			return Vec2D(homo.x / homo.z, homo.y / homo.z);
		}

		// input & gradInput
		// given h & dh/dx, return dp/dx = dp/dh * dh/dx
		static inline Vec2D gradproj(const Vec &homo, const Vec& gradhomo) {
			double hz_inv = 1.0 / homo.z;
			double hz_sqr_inv = 1.0 / sqr(homo.z);
			return Vec2D{gradhomo.x * hz_inv - gradhomo.z * homo.x * hz_sqr_inv,
									 gradhomo.y * hz_inv - gradhomo.z * homo.y * hz_sqr_inv };
		}

		static inline Vec proj2homo(const Vec2D &proj) {
			return Vec(proj.x, proj.y, 1);
		}
	}

	namespace cylindrical {
		static inline Vec2D homo2proj(const Vec &homo) {
			return Vec2D(atan2(homo.x, homo.z),
					homo.y / (hypot(homo.x, homo.z)));
		}

		static inline Vec proj2homo(const Vec2D &proj) {
			return Vec(sin(proj.x), proj.y, cos(proj.x));
		}
	}


	namespace spherical {
		// not scale-invariant!
    // after mult by -1
    // y <-  -pi - y if (x<0) else pi - y
    // x <- x \pm pi
		static inline Vec2D homo2proj(const Vec &homo) {
			return Vec2D(atan2(homo.x, homo.z),
					atan2(homo.y, hypot(homo.x, homo.z)));
		}

		// input & gradInput
		// given h & dh/dx, return dp/dx = dp/dh * dh/dx
		static inline Vec2D gradproj(const Vec &homo, const Vec& gradhomo) {
			double h_xz = homo.x * homo.x + homo.z * homo.z,
						 h_xz_r = sqrt(h_xz),
						 h_xyz_inv = 1.0 / (h_xz + homo.y * homo.y),
						 h_xz_inv = 1.0 / h_xz;
			return Vec2D{gradhomo.x * homo.z * h_xz_inv - gradhomo.z * homo.x * h_xz_inv,
									 -gradhomo.x * homo.x * homo.y * h_xyz_inv / h_xz_r
									 +gradhomo.y * h_xz_r * h_xyz_inv
									 -gradhomo.z * homo.y * homo.z * h_xyz_inv / h_xz_r};
		}

		static inline Vec proj2homo(const Vec2D &proj) {
			return Vec(sin(proj.x), tan(proj.y), cos(proj.x));
		}
	}
}
