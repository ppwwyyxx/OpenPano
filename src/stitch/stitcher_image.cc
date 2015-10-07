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
		Vec2D(-0.5, -0.5), Vec2D(-0.5, 0.5), Vec2D(0.5, -0.5), Vec2D(0.5, 0.5)};
	int refw = component[identity_idx].imgptr->width(),
			refh = component[identity_idx].imgptr->height();

	auto homo2proj = get_homo2proj();

	proj_min = Vec2D(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	proj_max = Vec2D(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
	for (auto& m : component) {
		Vec2D now_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
					now_max = now_min * (-1);
		for (auto& v : corner) {
			Vec homo = m.homo.trans(
					Vec2D(v.x * m.imgptr->width(), v.y * m.imgptr->height()));
			homo.x /= refw, homo.y /= refh;
			homo.x += 0.5 * homo.z, homo.y += 0.5 * homo.z;
			Vec2D t_corner = homo2proj(homo);
			t_corner.x *= refw, t_corner.y *= refh;
			now_min.update_min(t_corner);
			now_max.update_max(t_corner);
		}
		m.range = Range(now_min, now_max);
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
