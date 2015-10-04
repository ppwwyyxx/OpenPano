//File: blender.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include "lib/mat.h"
#include "lib/geometry.hh"
#include "lib/color.hh"

class BlenderBase {
	public:
		virtual ~BlenderBase() {}

		// upper_left: position of upper left corner of this image on target
		// orig_pos: maps each pixel on the target image to scaled
		//  	coordinate on original image,
		//  	or NaN if not on target image
		// img: the original image
		virtual void add_image(const Coor &upper_left,
				const Mat<Vec2D> &orig_pos, const Mat32f &img) = 0;

		virtual void run(Mat32f &target) = 0;
};

class LinearBlender : public BlenderBase {
	struct WeightedPixel { Color v; float w = 0; };
	struct Range {
		Coor min, max;
		bool contain(int r, int c) const {
			return (r >= min.y && r < max.y && c >= min.x && c < max.x);
		}
	};

	struct WeightedImage {
		Mat<WeightedPixel> mat;
		Range range;
		// We must have: range.max-min == mat.w,h
		WeightedImage(const Mat<WeightedPixel>& m, const Range& r):
			mat(m), range(r) {}
	};
	std::vector<WeightedImage> imgs;

	public:
		void add_image(const Coor &top_left,
				const Mat<Vec2D> &orig_pos, const Mat32f &img);

		void run(Mat32f &target);
};
