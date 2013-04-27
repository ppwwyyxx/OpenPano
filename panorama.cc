// File: panorama.cc
// Date: Sat Apr 27 20:29:52 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>
//
#include "panorama.hh"
#include "matcher.hh"
#include "utils.hh"
#include "sphere.hh"
#include "sift.hh"
#include "keypoint.hh"
#include "transformer.hh"
using namespace std;

imgptr Panorama::get() const {
	Matrix I(3, 3);
	I.get(0, 0) = I.get(1, 1) = I.get(2, 2) = 1;

	Cylinder cyl(std::max(imgs[0]->w, imgs[0]->h) * 2,
			Vec(0, imgs[0]->h / 2,
				std::max(imgs[0]->w, imgs[0]->h) * 2));

	Vec2D min(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
		 max(0, 0);
	int n = imgs.size();
	vector<Matrix> mat;
	mat.push_back(move(I));

	vector<Feature> feat1 = Panorama::get_feature(imgs[0]);
	REP(i, n - 1) {
		vector<Feature> feat2 = Panorama::get_feature(imgs[i + 1]);
		mat.push_back(get_transform(feat1, feat2));
		cout << mat[i + 1] << endl;
		feat1 = move(feat2);
	}
	REPL(i, 2, n) mat[i] = mat[i - 1].prod(mat[i]);

	/*
	 *REPL(i, 0, n) {
	 *    Vec2D corner[4] = {
	 *        Vec2D(0, 0), Vec2D(imgs[i]->w, 0),
	 *        Vec2D(0, imgs[i]->h), Vec2D(imgs[i]->w, imgs[i]->h)};
	 *    for (auto &v : corner) {
	 *        Vec2D newcorner = TransFormer::cal_project(mat[i], v);
	 *        min.update_min(Coor(floor(newcorner.x), floor(newcorner.y)));
	 *        max.update_max(Coor(ceil(newcorner.x), ceil(newcorner.y)));
	 *    }
	 *}
	 */
#pragma omp parallel for schedule(dynamic)
	REP(k, n) REP(i, imgs[k]->h) REP(j, imgs[k]->w) {
		Vec2D newcorner = TransFormer::cal_project(mat[k], Vec2D(j, i));
		newcorner = cyl.proj(newcorner);
		real_t hh = newcorner.x; // + cyl.center.y / 2;
		real_t ww = (M_PI - newcorner.y);
#pragma omp critical
		{
			min.update_min(Vec2D(ww, floor(hh)));
			max.update_max(Vec2D(ww, ceil(hh)));
		}
	}

	for_each(mat.begin(), mat.end(),
		[](Matrix & m) {
			Matrix inv(m.w, m.h);
			bool ok = m.inverse(inv);
			m_assert(ok);
			m = move(inv);
		});

	real_t initial_ang = min.x,
		   tot_ang = max.x - min.x;

	cout << "ini" << initial_ang << endl;
	cout << "tot" << tot_ang << endl;
	max.x *= cyl.r, min.x *= cyl.r;
	Coor size = toCoor(max - min);
	Vec2D offset(-min.x, -min.y);
	cout << "size: " << size << endl;
	cout << "offset" << offset << endl;;
	imgptr ret(new Img(size.x, size.y));
	ret->fill(Color::BLACK);

	HWTimer timer;
#pragma omp parallel for schedule(dynamic)
	REP(i, ret->h) REP(j, ret->w) {
		Vec2D final = Vec2D(j, i) - offset;
		real_t hh = final.y;
		real_t ww = initial_ang + (real_t)j / ret->w * tot_ang;
		final = cyl.proj_r(Vec2D(hh, M_PI - ww));
		cout << Vec2D(hh, ww) << " " << final << endl;
		vector<Color> blender;
		REP(k, n) {
			Vec2D old = TransFormer::cal_project(mat[k], final);
			if (between(old.x, 0, imgs[k]->w) && between(old.y, 0, imgs[k]->h))
				blender.push_back(imgs[k]->get_pixel(old.y, old.x));
		}
		if (!blender.size()) continue;
		Color finalc;
		for (auto &c : blender)
			finalc = finalc + c;
		finalc = finalc * ((real_t)1 / blender.size());
		ret->set_pixel(i, j, finalc);
	}
	print_debug("blend time: %lf secs\n", timer.get_sec());
	return ret;
}


Matrix Panorama::get_transform(const vector<Feature>& feat1, const vector<Feature>& feat2) {

	HWTimer timer;
	Matcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("match time: %lf secs\n", timer.get_sec());

	TransFormer transf(ret);
	return transf.get_transform();
}

vector<Feature> Panorama::get_feature(imgptr ptr) {
	ScaleSpace ss(ptr, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.work();
	return move(ex.features);
}
