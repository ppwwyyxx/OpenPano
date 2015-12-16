//File: blender.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "blender.hh"

#include <iostream>
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
}

void LinearBlender::run(Mat32f &target) {
	m_assert(target.channels() == 3);
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

						// TODO speedup. edge & interpolate are redundant
						if (!is_edge_color(img.img, r, c)){
							auto color = interpolate(img.img, r, c);
							// TODO decide which interpolation method to use
							// t.w = (0.5 - fabs(p.x / img.width() - 0.5)) * (0.5 - fabs(p.y / img.height() - 0.5));
							// x-axis linear interpolation
							float w = 0.5 - fabs(c / img.img.width() - 0.5);

							isum += color * w;
							wsum += w;
						}
					}
				}
			if (wsum)	// keep original Color::NO
				(isum / wsum).write_to(row + j * 3);
		}
	}
}

}
