//File: stitcher_image.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include "stitcher_image.hh"
#include "projection.hh"
#include "lib/config.hh"
#include "lib/timer.hh"
#include "lib/imgproc.hh"
#include "blender.hh"
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

Mat32f ConnectedImages::blend() const {
	GuardedTimer tm("blend()");
	// it's hard to do coordinates.......
	int refw = component[identity_idx].imgptr->width(),
			refh = component[identity_idx].imgptr->height();
	auto homo2proj = get_homo2proj();
	auto proj2homo = get_proj2homo();

	Vec2D id_img_range = homo2proj(Vec(refw, refh, 1)) - homo2proj(Vec(0, 0, 1));
	cout << "projmin:" << proj_range.min << "projmax" << proj_range.max << endl;
	if (proj_method != ProjectionMethod::flat) {
		id_img_range = homo2proj(Vec(1,1,1)) - homo2proj(Vec(0,0,1));
		//id_img_range.x *= refw, id_img_range.y *= refh;
		// this yields better aspect ratio in the result.
		id_img_range.x *= (refw * 1.0 / refh);
	}

	Vec2D proj_min = proj_range.min;
	double x_len = proj_range.max.x - proj_min.x,
				 y_len = proj_range.max.y - proj_min.y,
				 x_per_pixel = id_img_range.x / refw,
				 y_per_pixel = id_img_range.y / refh,
				 target_width = x_len / x_per_pixel,
				 target_height = y_len / y_per_pixel;

	Coor size(target_width, target_height);
	print_debug("Final Image Size: (%d, %d)\n", size.x, size.y);
	if (max(size.x, size.y) > 30000 || size.x * size.y > 600000000)
		error_exit("Result too large. Something must be wrong\n");

	auto scale_coor_to_img_coor = [&](Vec2D v) {
		v = v - proj_min;
		v.x /= x_per_pixel, v.y /= y_per_pixel;
		return Coor(v.x, v.y);
	};

	// blending
	Mat32f ret(size.y, size.x, 3);
	fill(ret, Color::NO);

	LinearBlender blender;
	for (auto& cur : component) {
		Coor top_left = scale_coor_to_img_coor(cur.range.min);
		Coor bottom_right = scale_coor_to_img_coor(cur.range.max);

		blender.add_image(top_left, bottom_right, *cur.imgptr, [=,&cur](Coor t) -> Vec2D {
			Vec2D c(t.x * x_per_pixel + proj_min.x,
							t.y * y_per_pixel + proj_min.y);
			Vec homo = proj2homo(Vec2D(c.x, c.y));
			Vec2D orig = cur.homo_inv.trans_normalize(homo);
			return orig;
		});
	}
	//if (DEBUG_OUT) blender.debug_run(size.x, size.y);
	blender.run(ret);
	return ret;

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
