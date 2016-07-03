//File: multiband.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once

#include "blender.hh"
#include "lib/matrix.hh"

namespace pano {

class MultiBandBlender : public BlenderBase {
	struct WeightedPixel {
		float w;
		Color c;

		WeightedPixel() {}
		WeightedPixel(float v): w(v), c(v,v,v) {}
		WeightedPixel(float w, const Color& c): w(w), c(c) {}

		WeightedPixel operator * (float v) const { return WeightedPixel{w * v, c * v}; }
		void operator += (const WeightedPixel& p) { w += p.w; c += p.c; }
	};

	struct ImageToBlend {
		Range range;
		Mat<WeightedPixel> img;		// a RoI in target image, starting from range.min

		const float& weight_on_target(int x, int y) const {
			// x, y: coordinate on target
			return img.at(y - range.min.y, x - range.min.x).w;
		}

		float& weight_on_target(int x, int y) {
			return img.at(y - range.min.y, x - range.min.x).w;
		}

		const Color& color_on_target(int x, int y) const {
			return img.at(y - range.min.y, x - range.min.x).c;
		}
	};

	std::vector<ImageToBlend> images;
	std::vector<ImageToBlend> next_lvl_images;

	void update_weight_map();
	// build next level weights from images to next_lvl_images
	void create_next_level(int level);
	// save image and weight from next_lvl_images
	void debug_level(int level) const;


	Coor target_size{0, 0};
	int band_level;

	public:
	MultiBandBlender(int band_level):
		band_level(band_level) {} // default: 5?

	void add_image(
			const Coor& upper_left,
			const Coor& bottom_right,
			const ImageMeta &img,
			std::function<Vec2D(Coor)>) override;

	Mat32f run() override;
};

}	// namespace pano
