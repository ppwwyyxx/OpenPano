//File: blender.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "blender.hh"

#include <iostream>
#include "lib/imgproc.hh"
#include "lib/timer.hh"
using namespace std;

void LinearBlender::add_image(const Coor &top_left,
		const Mat<Vec2D> &orig_pos, const Mat32f &img) {
	TotalTimer tm("blender_add");
	int w = orig_pos.width(), h = orig_pos.height();
	Mat<WeightedPixel> mat(h, w, 1);
	REP(i, h) {
		const Vec2D* orig_row = orig_pos.ptr(i);
		WeightedPixel* mat_row = mat.ptr(i);
		REP(j, w) {
			const Vec2D& p = *(orig_row + j);	// coordinate on original image
			if (!p.isNaN()) {
				float r = p.y * img.height(), c = p.x * img.width();
				// since p < 1, r and c should be valid coor
				// TODO speedup. edge & interpolate are redundant
				if (!is_edge_color(img, r, c)){
					WeightedPixel* t = mat_row + j;
					t->v = interpolate(img, r, c);
					// t.w = (0.5 - fabs(p.x - 0.5)) * (0.5 - fabs(p.y - 0.5));
					// x-axis linear interpolation
					t->w = max(0.5 - fabs(p.x - 0.5), 0.1);
				}
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
			if (wsum)	// keep original Color::NO
				(isum / wsum).write_to(row + j * 3);
		}
	}
}
