//File: multiband.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once

#include "blender.hh"
#include "lib/matrix.hh"

namespace pano {

class MultiBandBlender : public BlenderBase {
	struct WeightedPixel {
		Color c;
		float w;

		WeightedPixel() {}
		WeightedPixel(float v): c(v,v,v), w(v) {}
		WeightedPixel(float w, const Color& c): c(c), w(w) {}

		WeightedPixel operator * (float v) const { return WeightedPixel{w * v, c * v}; }
		void operator += (const WeightedPixel& p) { w += p.w; c += p.c; }
	};

	struct Mask2D {
		std::vector<bool> mask;
		int w;

		bool get(int i, int j) const { return mask[i * w + j]; }
		void set(int i, int j) { mask[i * w + j] = true; }
		Mask2D(int h, int w):
			mask(h * w, false), w{w} {}
	};

	struct ImageMeta {
		Range range;
		Mask2D mask;		// 1: invalid
	};

	struct ImageToBlend {
		Mat<WeightedPixel> img;		// a RoI in target image, starting from range.min
		const ImageMeta& meta;

		float weight_on_target(int x, int y) const {
			// x, y: coordinate on target
			return img.at(y - meta.range.min.y, x - meta.range.min.x).w;
		}

		float& weight_on_target(int x, int y) {
			return img.at(y - meta.range.min.y, x - meta.range.min.x).w;
		}

		const Color& color_on_target(int x, int y) const {
			return img.at(y - meta.range.min.y, x - meta.range.min.x).c;
		}

		bool valid_on_target(int x, int y) const {
			x -= meta.range.min.x, y -= meta.range.min.y;
			return not meta.mask.get(y, x);
		}

	};

	std::vector<ImageToAdd> add_images;
	std::vector<ImageMeta> image_metas;
	std::vector<ImageToBlend> images;
	std::vector<ImageToBlend> next_lvl_images;

	void create_first_level();
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
			ImageRef &img,
			std::function<Vec2D(Coor)>) override;

	Mat32f run() override;
};

}	// namespace pano
