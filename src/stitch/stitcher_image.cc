//File: stitcher_image.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>
#include "stitcher_image.hh"
#include "projection.hh"
#include "lib/config.hh"
#include <cassert>

using namespace std;
using namespace config;

namespace pano {

void ConnectedImages::shift_all_homo() {
	int mid = identity_idx;
	Homography t2 = Homography::get_translation(
			component[mid].imgptr->width() * 0.5,
			component[mid].imgptr->height() * 0.5);
	REP(i, (int)component.size())
		if (i != mid) {
			Homography t1 = Homography::get_translation(
					component[i].imgptr->width() * 0.5,
					component[i].imgptr->height() * 0.5);
			component[i].homo = t2 * component[i].homo * t1.inverse();
		}
}

void ConnectedImages::calc_inverse_homo() {
	for (auto& m : component)
		m.homo_inv = m.homo.inverse();
}

void ConnectedImages::update_proj_range() {
	vector<Vec2D> corner;
	const static int CORNER_SAMPLE = 100;
	REP(i, CORNER_SAMPLE) REP(j, CORNER_SAMPLE)
		corner.emplace_back((double)i / CORNER_SAMPLE, (double)j / CORNER_SAMPLE);

	auto homo2proj = get_homo2proj();

	Vec2D proj_min = Vec2D::max();
	Vec2D proj_max = proj_min * (-1);
	for (auto& m : component) {
		Vec2D now_min(numeric_limits<double>::max(), std::numeric_limits<double>::max()),
					now_max = now_min * (-1);
		for (auto v : corner) {
			Vec homo = m.homo.trans(
					Vec2D(v.x * m.imgptr->width(), v.y * m.imgptr->height()));
			Vec2D t_corner = homo2proj(homo);
			now_min.update_min(t_corner);
			now_max.update_max(t_corner);
		}
		// assume no image has FOV > 180
		// XXX TODO ugly
		if (proj_method != ProjectionMethod::flat &&
				now_max.x - now_min.x > M_PI) {
			// head and tail
			now_min = Vec2D(numeric_limits<double>::max(), std::numeric_limits<double>::max());
			now_max = now_min * (-1);
			for (auto v : corner) {
				Vec homo = m.homo.trans(
						Vec2D(v.x * m.imgptr->width(), v.y * m.imgptr->height()));
				Vec2D t_corner = homo2proj(homo);
				if (t_corner.x < 0) t_corner.x += 2*M_PI;
				now_min.update_min(t_corner);
				now_max.update_max(t_corner);
			}
		}
		m.range = Range(now_min, now_max);
		proj_min.update_min(now_min);
		proj_max.update_max(now_max);
		print_debug("Range: (%lf,%lf)~(%lf,%lf)\n",
				m.range.min.x, m.range.min.y,
				m.range.max.x, m.range.max.y);
	}
	if (proj_method != ProjectionMethod::flat) {
		// TODO keep everything inside 2 * pi
		// doesn't seem to be trivial. need to maintain range of each component
	}
	proj_range.min = proj_min, proj_range.max = proj_max;
}

homo2proj_t ConnectedImages::get_homo2proj() const {
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

proj2homo_t ConnectedImages::get_proj2homo() const {
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

}
