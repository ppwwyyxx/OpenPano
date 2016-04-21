//File: blender.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "blender.hh"

#include <iostream>
#include "lib/config.hh"
#include "lib/imgproc.hh"
#include "lib/timer.hh"
using namespace std;

namespace pano {

void LinearBlender::add_image(
			const Coor& upper_left,
			const Coor& bottom_right,
			const Mat32f &img,
			std::function<Vec2D(Coor)> coor_func) {
	images.emplace_back(ImageToBlend{Range{upper_left, bottom_right}, img, coor_func});
	target_size.update_max(bottom_right);
}

Mat32f LinearBlender::run() {
	Mat32f target(target_size.y, target_size.x, 3);
	fill(target, Color::NO);
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < target.height(); i ++) {
		float *row = target.ptr(i);
		for (int j = 0; j < target.width(); j ++) {
			Color isum = Color::BLACK;
			float wsum = 0;
			for (auto& img : images)
				if (img.range.contain(i, j)) {
					Vec2D img_coor = img.map_coor(i, j);
					if (!img_coor.isNaN()) {
						float r = img_coor.y, c = img_coor.x;
						auto color = interpolate(img.img, r, c);
						if (color.x < 0) continue;
						float w;
						if (config::ORDERED_INPUT)
							// x-axis linear interpolation
							w = 0.5 - fabs(c / img.img.width() - 0.5);
						else
						  w = (0.5 - fabs(c / img.img.width() - 0.5)) * (0.5 - fabs(r / img.img.height() - 0.5));

						isum += color * w;
						wsum += w;
					}
				}
			if (wsum > 0)	// keep original Color::NO
				(isum / wsum).write_to(row + j * 3);
		}
	}
	return target;
}

}
