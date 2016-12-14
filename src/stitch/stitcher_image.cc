//File: stitcher_image.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <cassert>
#include <memory>
#include "stitcher_image.hh"
#include "projection.hh"
#include "lib/config.hh"
#include "lib/timer.hh"
#include "multiband.hh"
#include "lib/imgproc.hh"
#include "blender.hh"

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
		corner.emplace_back((double)i / CORNER_SAMPLE - 0.5, (double)j / CORNER_SAMPLE - 0.5);

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
		m.range = Range(now_min, now_max);
		proj_min.update_min(now_min);
		proj_max.update_max(now_max);
		print_debug("Range: (%lf,%lf)~(%lf,%lf)\n",
				m.range.min.x, m.range.min.y,
				m.range.max.x, m.range.max.y);
	}
	proj_range.min = proj_min, proj_range.max = proj_max;
}

Vec2D ConnectedImages::get_final_resolution() const {
	cout << "projmin: " << proj_range.min << ", projmax: " << proj_range.max << endl;

	int refw = component[identity_idx].imgptr->width(),
			refh = component[identity_idx].imgptr->height();
	auto homo2proj = get_homo2proj();
  const Homography& identity_H = component[identity_idx].homo;
  // transform corners to point in space to estimate range

  Vec id_img_corner2 = identity_H.trans(Vec2D{refw/2.0, refh/2.0}),
      id_img_corner1 = identity_H.trans(Vec2D{-refw/2.0, -refh/2.0});
  // the range of the identity image
  Vec2D id_img_range = homo2proj(id_img_corner2) - homo2proj(id_img_corner1);
  cout << "Identity projection range: " << id_img_range << endl;
  if (proj_method != ProjectionMethod::flat) {
    if (id_img_range.x < 0)
      id_img_range.x = 2 * M_PI + id_img_range.x;
    if (id_img_range.y < 0)
      id_img_range.y = M_PI + id_img_range.y;
  }

	Vec2D resolution = id_img_range / Vec2D(refw, refh),		// output-x-per-input-pixel, y-per-pixel
				target_size = proj_range.size() / resolution;
	double max_edge = max(target_size.x, target_size.y);
  print_debug("Target Image Size: (%lf, %lf)\n", target_size.x, target_size.y);
	if (max_edge > 80000 || target_size.x * target_size.y > 1e9)
		error_exit("Target size too large. Looks like a stitching failure!\n");
	// resize the result
	if (max_edge > MAX_OUTPUT_SIZE) {
		float ratio = max_edge / MAX_OUTPUT_SIZE;
		resolution *= ratio;
	}
  print_debug("Resolution: %lf,%lf\n", resolution.x, resolution.y);
	return resolution;
}

Mat32f ConnectedImages::blend() const {
	GuardedTimer tm("blend()");
	// it's hard to do coordinates.......
	auto proj2homo = get_proj2homo();
	Vec2D resolution = get_final_resolution();

	Vec2D size_d = proj_range.size() / resolution;
	Coor size(size_d.x, size_d.y);
	print_debug("Final Image Size: (%d, %d)\n", size.x, size.y);

	auto scale_coor_to_img_coor = [&](Vec2D v) {
		v = (v - proj_range.min) / resolution;
		return Coor(v.x, v.y);
	};

	// blending
	std::unique_ptr<BlenderBase> blender;
	if (MULTIBAND > 0)
		blender.reset(new MultiBandBlender{MULTIBAND});
	else
		blender.reset(new LinearBlender);
	for (auto& cur : component) {
		Coor top_left = scale_coor_to_img_coor(cur.range.min);
		Coor bottom_right = scale_coor_to_img_coor(cur.range.max);

		blender->add_image(top_left, bottom_right, *cur.imgptr,
				[=,&cur](Coor t) -> Vec2D {
					Vec2D c = Vec2D(t.x, t.y) * resolution + proj_range.min;
					Vec homo = proj2homo(Vec2D(c.x, c.y));
					Vec ret = cur.homo_inv.trans(homo);
					if (ret.z < 0)
						return Vec2D{-10, -10};	// was projected to the other side of the lens, discard
					double denom = 1.0 / ret.z;
					return Vec2D{ret.x*denom, ret.y*denom}
                + cur.imgptr->shape().center();
				});
	}
	//dynamic_cast<LinearBlender*>(blender.get())->debug_run(size.x, size.y);	// for debug
	return blender->run();
}

}
