//File: blender.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include <functional>
#include "lib/mat.h"
#include "lib/geometry.hh"
#include "lib/color.hh"

namespace pano {

class BlenderBase {
	public:
		virtual ~BlenderBase() {}

		// upper_left/bottom_right: position of the two corners of img on result image
		// coor_func: the function maps from target coordinate to original image coordinate.
		virtual void add_image(
				const Coor& upper_left,
				const Coor& bottom_right,
				const Mat32f &img,
				std::function<Vec2D(Coor)> coor_func) = 0;

		virtual void run(Mat32f &target) = 0;
};

class LinearBlender : public BlenderBase {
	struct Range {
		Coor min, max;
		bool contain(int r, int c) const {
			return (r >= min.y && r < max.y && c >= min.x && c < max.x);
		}
	};

	struct ImageToBlend {
		Range range;
		const Mat32f& img;
		std::function<Vec2D(Coor)> coor_func;

		Vec2D map_coor(int r, int c) const {
			auto ret = coor_func(Coor(c, r));
			if (ret.x < 0 || ret.x >= img.width() || ret.y < 0 || ret.y >= img.height())
				ret = Vec2D::NaN();
			return ret;
		}
	};
	std::vector<ImageToBlend> images;

	public:
		void add_image(
				const Coor& upper_left,
				const Coor& bottom_right,
				const Mat32f &img,
				std::function<Vec2D(Coor)>) override;

		void run(Mat32f &target) override;

		// render each component, for debug
		void debug_run(int w, int h);
};

}
