//File: blender.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include <functional>
#include "lib/mat.h"
#include "lib/geometry.hh"
#include "lib/color.hh"
#include "imageref.hh"
#include "common/common.hh"
namespace pano {

class BlenderBase {
	public:
		BlenderBase() = default;
		virtual ~BlenderBase() {}

		struct Range {
			Coor min, max;	// min, max are both inclusive
			bool contain(int r, int c) const {
				return (r >= min.y && r <= max.y
						&& c >= min.x && c <= max.x);
			}
			int width() const { return max.x - min.x + 1; }
			int height() const { return max.y - min.y + 1; }

			friend std::ostream& operator << (std::ostream& os, const Range& s) {
				os << "min=" << s.min << ",max=" << s.max;
				return os;
			}
		};

		struct ImageToAdd {
			Range range;
			ImageRef& imgref;
			std::function<Vec2D(Coor)> coor_func;

			Vec2D map_coor(int r, int c) const {
				auto ret = coor_func(Coor(c, r));
				if (ret.x < 0 || ret.x >= imgref.width() || ret.y < 0 || ret.y >= imgref.height())
					ret = Vec2D::NaN();
				return ret;
			}
		};

		BlenderBase(const BlenderBase&) = delete;
		BlenderBase& operator = (const BlenderBase&) = delete;

		// upper_left/bottom_right: range of img on result image
		// coor_func: the function maps from target coordinate to original image coordinate.
		virtual void add_image(
				const Coor& upper_left,
				const Coor& bottom_right,
				ImageRef &img,
				std::function<Vec2D(Coor)> coor_func) = 0;

		virtual Mat32f run() = 0;
};

class LinearBlender : public BlenderBase {
	std::vector<ImageToAdd> images;

	Coor target_size{0, 0};

	public:
	void add_image(
			const Coor& upper_left,
			const Coor& bottom_right,
			ImageRef &img,
			std::function<Vec2D(Coor)>) override;

	Mat32f run() override;

	// render each component, for debug
	void debug_run(int w, int h);
};

}
