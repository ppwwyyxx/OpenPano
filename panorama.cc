// File: panorama.cc
// Date: Wed May 01 01:44:36 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <fstream>
#include "panorama.hh"
#include "matcher.hh"
#include "utils.hh"
#include "cylinder.hh"
#include "keypoint.hh"
#include "transformer.hh"
using namespace std;

imgptr Panorama::get()
{ return get_trans(); }

imgptr Panorama::get_trans() {
	Matrix I = Matrix::I(3);

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

	ofstream fout("matrix.txt");
	REP(k, n) fout << mat[k] << endl;
	fout.close();

	REPL(i, 2, n) mat[i] = mat[i - 1].prod(mat[i]);

	REP(k, 1)
		straighten(mat);
	straighten_simple(0, n, mat);


	ofstream fout2("matrix_2.txt");
	REP(k, n) fout2 << mat[k] << endl;
	fout2.close();

	REPL(i, 0, n) {
	    Vec2D corner[4] = {
	        Vec2D(0, 0), Vec2D(imgs[i]->w, 0),
	        Vec2D(0, imgs[i]->h), Vec2D(imgs[i]->w, imgs[i]->h)};
	    for (auto &v : corner) {
	        Vec2D newcorner = TransFormer::cal_project(mat[i], v);
	        min.update_min(Vec2D(floor(newcorner.x), floor(newcorner.y)));
	        max.update_max(Vec2D(ceil(newcorner.x), ceil(newcorner.y)));
	    }
	}

	for_each(mat.begin(), mat.end(),
			[](Matrix & m) {
			Matrix inv(m.w, m.h);
			bool ok = m.inverse(inv);
			m_assert(ok);
			m = move(inv);
			});


	PP("min: ", min);
	PP("max: ", max);
	Vec2D diff = max - min;
	Coor size = toCoor(diff);

	Vec2D offset = min * (-1);
	PP("size: ", size);
	PP("offset", offset);

	imgptr ret(new Img(size.x, size.y));
	ret->fill(Color::BLACK);

	HWTimer timer;
#pragma omp parallel for schedule(dynamic)
	REP(i, ret->h) REP(j, ret->w) {
		Vec2D final = (Vec2D(j, i) - offset);
		vector<Color> blender;
		REP(k, n) {
			Vec2D old = TransFormer::cal_project(mat[k], final);
			if (between(old.x, 0, imgs[k]->w) && between(old.y, 0, imgs[k]->h)) {
				if (imgs[k]->is_black_edge(old.y, old.x)) continue;
				blender.push_back(imgs[k]->get_pixel(old.y, old.x));
			}
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

vector<Feature> Panorama::get_feature(imgptr& ptr) {
	ScaleSpace ss(ptr, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.work();
	Panorama::warp(ptr, ex.features);
	return move(ex.features);
}

void Panorama::warp(imgptr& img, vector<Feature>& ft) {
	int r = max(img->w, img->h) / 2;
	Vec cen(img->w / 2, img->h / 2 * 1.2, r * 2);
	CylProject cyl(r, cen, r );
	img = cyl.project(img, ft);
}

void Panorama::straighten_simple(int s, int t, vector<Matrix>& mat) const {
	int n = imgs.size();
	m_assert(between(s, 0, n + 1) && between(t, 0, n + 1));
	Vec2D center2 = imgs[t - 1]->get_center();
	center2 = TransFormer::cal_project(mat[t - 1], center2);
	Vec2D center1 = TransFormer::cal_project(mat[s], imgs[s]->get_center());
	real_t dydx = (center2.y - center1.y) / (center2.x - center1.x);
	Matrix S = Matrix::I(3);
	S.get(1, 0) = dydx;
	Matrix Sinv(3, 3);
	S.inverse(Sinv);
	REPL(i, 0, n)
		mat[i] = Sinv.prod(mat[i]);
}

void Panorama::straighten(vector<Matrix>& mat) const {
	int n = mat.size();

	vector<Vec2D> centers;
	REP(k, n)
		centers.push_back(TransFormer::cal_project(mat[k], imgs[k]->get_center()));
	Vec2D kb = Panorama::line_fit(centers);
	P(kb);
	Matrix shift = Panorama::shift_to_line(centers, kb);
	P(shift);
	for (auto& i : mat) i = shift.prod(i);
}

Vec2D Panorama::line_fit(const std::vector<Vec2D>& pts) {
	int n = pts.size();

	Matrix left(2, n);
	Matrix right(1, n);
	REP(k, n) {
		left.get(k, 0) = pts[k].x, left.get(k, 1) = 1;
		right.get(k, 0) = pts[k].y;
	}

	Matrix res(0, 0);
	if (!left.solve_overdetermined(res, right)) {
		cout << "line_fit solve failed" << endl;
		return Vec2D(0, 0);
	}
	return Vec2D(res.get(0, 0), res.get(1, 0));
}

Matrix Panorama::shift_to_line(const vector<Vec2D>& ptr, const Vec2D& line) {
	int n = ptr.size();
	m_assert(n >= 4);
	Matrix left(4, 2 * n);
	Matrix right(1, 2 * n);
	REP(k, n) {
		auto & nowp = ptr[k];
		real_t targetx = (nowp.x - (line.y - nowp.y) / line.x) / 2;
		real_t targety = line.x * targetx + line.y;
		left.get(2 * k, 0) = nowp.x;
		left.get(2 * k, 1) = nowp.y;
		left.get(2 * k, 2) = left.get(2 * k, 3) = 0;
		right.get(2 * k, 0) = targetx;

		left.get(2 * k + 1, 0) = left.get(2 * k + 1, 1) = 0;
		left.get(2 * k + 1, 2) = nowp.x;
		left.get(2 * k + 1, 3) = nowp.y;
		right.get(2 * k + 1, 0) = targety;
	}
	Matrix res(1, 4);
	if (!left.solve_overdetermined(res, right)) {
		cout << "line_fit solve failed" << endl;
		return move(res);
	}
	/*
	 *Matrix tmp(2, 2);
	 *REP(i, 4)
	 *    tmp.get(i / 2, i % 2) = res.get(i, 0);
	 *REP(k, n) {
	 *    P(ptr[k]);
	 *    cout << "turned to";
	 *    Matrix old(1, 2);
	 *    old.get(0, 0) = ptr[k].x, old.get(1, 0) = ptr[k].y;
	 *    P(tmp.prod(old));
	 *}
	 */
	Matrix ret(3, 3);
	ret.get(0, 0) = res.get(0, 0);
	ret.get(0, 1) = res.get(1, 0);
	ret.get(1, 0) = res.get(2, 0);
	ret.get(1, 1) = res.get(3, 0);
	ret.get(2, 2) = 1;
	for (auto &i : ptr) {
		Vec2D project = TransFormer::cal_project(ret, i);
		cout << project << " ==?" << (line.x * project.x + line.y) << endl;
	}
	return move(ret);
}
