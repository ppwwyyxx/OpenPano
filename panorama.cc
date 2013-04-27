// File: panorama.cc
// Date: Sat Apr 27 21:57:32 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <fstream>
#include "panorama.hh"
#include "matcher.hh"
#include "utils.hh"
#include "sphere.hh"
#include "sift.hh"
#include "keypoint.hh"
#include "transformer.hh"
using namespace std;

extern bool TEMPDEBUG;
imgptr Panorama::get() const {
	Matrix I(3, 3);
	I.get(0, 0) = I.get(1, 1) = I.get(2, 2) = 1;

	Cylinder cyl(std::max(imgs[1]->w, imgs[1]->h) * 1.5,
			Vec(imgs[0]->w, imgs[0]->h / 2, std::max(imgs[1]->w, imgs[1]->h) * 1.5));

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
	vector<Matrix> oldmat = mat;
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
	ofstream fout("matrix.txt");
	REP(k, n) fout << mat[k] << endl;
	fout.close();
/*
 *#pragma omp parallel for schedule(dynamic)
 */
	REP(k, n)
		REP(i, imgs[k]->h) REP(j, imgs[k]->w) {
			Vec2D newcorner = Vec2D(j, i);
			for (int t = k; t > 0; t --)
				newcorner = TransFormer::cal_project(oldmat[t], newcorner);
		/*
		 *Vec2D newcorner = TransFormer::cal_project(mat[k], Vec2D(j, i));
		 *if (k > 0) {
		 *    newcorner = TransFormer::cal_project(oldmat[k], Vec2D(j, i));
		 *    newcorner = TransFormer::cal_project(mat[k - 1], newcorner);
		 *}
		 */
		newcorner = cyl.proj(newcorner);
		real_t hh = newcorner.x; // + cyl.center.y / 2;
		real_t ww = (M_PI - newcorner.y);
		if (newcorner.y < 1e-3) {
			/*
			 *P(newcorner);
			 */
			P(Vec2D(j, i));
			Vec2D test = TransFormer::cal_project(oldmat[k], Vec2D(j, i));
			test = TransFormer::cal_project(oldmat[k - 1], test);
			test = TransFormer::cal_project(oldmat[k - 2], test);
			test = TransFormer::cal_project(oldmat[k - 3], test);
			test = TransFormer::cal_project(oldmat[k - 4], test);
			P(test);
			cout << oldmat[k - 5] << endl;
			TEMPDEBUG = true;
			P(TransFormer::cal_project(oldmat[k - 5], test));
			m_assert(false);
		}
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
	if (initial_ang < 1e-3) initial_ang = 0;
	m_assert(tot_ang < M_PI);
	PP("initial: ", initial_ang);
	PP("totang: ", tot_ang);

	Coor size(tot_ang * cyl.r, max.y - min.y);
	real_t heightoffset = -min.y;
	PP("size: ", size);
	PP("offset", heightoffset);

	imgptr ret(new Img(size.x, size.y));
	ret->fill(Color::BLACK);

	HWTimer timer;
#pragma omp parallel for schedule(dynamic)
	REP(i, ret->h) REP(j, ret->w) {
		real_t hh = i - heightoffset;
		real_t ww = initial_ang + (real_t)j / ret->w * tot_ang;
		Vec2D final = cyl.proj_r(Vec2D(hh, M_PI - ww));
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
