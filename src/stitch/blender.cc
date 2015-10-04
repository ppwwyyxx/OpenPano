//File: blender.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "blender.hh"

#include <iostream>
#include "lib/imgproc.hh"
using namespace std;

// TODO blender has a lot to speed up

void LinearBlender::add_image(const Coor &top_left,
		const Mat<Vec2D> &orig_pos, const Mat32f &img) {
	int w = orig_pos.width(), h = orig_pos.height();
	Mat<WeightedPixel> mat(h, w, 1);
	REP(i, h) REP(j, w) {
		const Vec2D& p = orig_pos.at(i, j);	// coordinate on original image
		auto& t = mat.at(i, j);
		if (!p.isNaN()) {
			float r = p.y * img.height(), c = p.x * img.width();
			// since p < 1, r and c should be valid coor
			if (!is_edge_color(img, r, c)){
				t.v = interpolate(img, r, c);
				//t.w = (0.5 - fabs(p.x - 0.5)) * (0.5 - fabs(p.y - 0.5));
				// x-axis linear interpolation
				t.w = max(0.5 - fabs(p.x - 0.5), 0.1);
			}
		}
	}
	imgs.emplace_back(mat,
			Range{top_left, top_left + Coor(w, h)});
}

void LinearBlender::run(Mat32f &target) {
	m_assert(target.channels() == 3);
	for (int i = 0; i < target.height(); i ++) {
		float *row = target.ptr(i);
		for (int j = 0; j < target.width(); j ++) {
			Color isum = Color::BLACK;
			float wsum = 0;
			for (auto &img: imgs)
				if (img.range.contain(i, j)) {
					auto &w = img.mat.at(i - img.range.min.y,
							j - img.range.min.x);
					if (w.w > 0) {
						isum += w.v * w.w;
						wsum += w.w;
					}
				}
			if (wsum)
				(isum / wsum).write_to(row + j * 3);
		}
	}
}
