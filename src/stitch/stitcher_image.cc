//File: stitcher_image.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "stitcher.hh"
#include <cassert>

void ConnectedImages::calc_inverse_homo() {
	for (auto& m : component)	 {
		bool ok = m.homo.inverse(m.homo_inv);
		m_assert(ok);
		m.homo_inv.normalize();
	}
}

void ConnectedImages::update_proj_range() {
	static Vec2D corner[4] = {
		Vec2D(0, 0), Vec2D(0, 1), Vec2D(1, 0), Vec2D(1, 1)};

	auto homo2proj = get_homo2proj();

	proj_min = Vec2D(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	proj_max = Vec2D(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
	proj_ranges.clear();
	for (auto& m : component) {
		Vec2D now_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
					now_max = now_min * (-1);
		for (auto& v : corner) {
			//Vec2D t_corner = m.homo.trans2d(v);
			Vec2D t_corner = homo2proj(m.homo.trans(v));
			now_min.update_min(t_corner);
			now_max.update_max(t_corner);
		}
		proj_ranges.emplace_back(now_min, now_max);
		proj_min.update_min(now_min);
		proj_max.update_max(now_max);
	}
}

projector::homo2proj_t ConnectedImages::get_homo2proj() const {
	using namespace projector;
	switch (proj_method) {
		case ProjectionMethod::flat:
			return flat::homo2proj;
		case ProjectionMethod::cylindrical:
			return cylindrical::homo2proj;
		case ProjectionMethod::spherical:
			return spherical::homo2proj;
	}
	assert(false);
}

projector::proj2homo_t ConnectedImages::get_proj2homo() const {
	using namespace projector;
	switch (proj_method) {
		case ProjectionMethod::flat:
			return flat::proj2homo;
		case ProjectionMethod::cylindrical:
			return cylindrical::proj2homo;
		case ProjectionMethod::spherical:
			return spherical::proj2homo;
	}
	assert(false);
}
