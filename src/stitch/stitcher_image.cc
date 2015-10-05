//File: stitcher_image.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "stitcher.hh"
using namespace projector;

void ConnectedImages::calc_inverse_homo() {
	for (auto& m : component)	 {
		bool ok = m.homo.inverse(m.homo_inv);
		m_assert(ok);
	}
}

void ConnectedImages::update_proj_range() {
	static Vec2D corner[4] = {
		Vec2D(0, 0), Vec2D(0, 1), Vec2D(1, 0), Vec2D(1, 1)};

	proj_min = Vec2D(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	proj_max = proj_min * (-1);
	proj_ranges.clear();
	for (auto& m : component) {
		Vec2D now_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
					now_max = now_min * (-1);
		for (auto& v : corner) {
			Vec2D t_corner = m.homo.trans2d(v);
			//Vec2D t_corner = cylindrical::homo2proj(m.homo.trans(v));
			now_min.update_min(t_corner);
			now_max.update_max(t_corner);
		}
		proj_ranges.emplace_back(now_min, now_max);
		proj_min.update_min(now_min);
		proj_max.update_max(now_max);
	}
}
