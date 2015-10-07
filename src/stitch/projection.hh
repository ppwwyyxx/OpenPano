//File: projection.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once

#include "lib/geometry.hh"

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
			return Vec2D(atan2(coord.x, coord.z),
					coord.y / (hypot(coord.x, coord.z)));
		}

		static inline Vec proj2homo(const Vec2D &coord) {
			return Vec(sin(coord.x), coord.y, cos(coord.x));
		}
	}


	namespace spherical {
		static inline Vec2D homo2proj(const Vec &coord) {
			return Vec2D(atan2(coord.x, coord.z),
					atan2(coord.y, hypot(coord.x, coord.z)));
		}

		static inline Vec proj2homo(const Vec2D &coord) {
			return Vec(sin(coord.x), tan(coord.y), cos(coord.x));
		}
	}
}
