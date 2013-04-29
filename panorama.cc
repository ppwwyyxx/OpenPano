// File: panorama.cc
// Date: Tue Apr 30 02:02:11 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <fstream>
#include "panorama.hh"
#include "matcher.hh"
#include "utils.hh"
/*
 *#include "sphere.hh"
 */
#include "sift.hh"
#include "keypoint.hh"
#include "transformer.hh"
using namespace std;

imgptr Panorama::get() const {
	return get_trans();
}

imgptr Panorama::get_trans() const {
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

	/*
	 *ofstream fout("matrix.txt");
	 *REP(k, n) fout << mat[k] << endl;
	 *fout.close();
	 */

	REPL(i, 2, n) mat[i] = mat[i - 1].prod(mat[i]);

	straighten(mat);

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

vector<Feature> Panorama::get_feature(imgptr ptr) {
	ScaleSpace ss(ptr, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.work();
	return move(ex.features);
}

void Panorama::straighten_first_last(vector<Matrix>& mat) const {
	int n = imgs.size();
	Vec2D center2 = imgs[n - 1]->get_center();
	center2 = TransFormer::cal_project(mat[n - 1], center2);
	Vec2D center = TransFormer::cal_project(mat[0], imgs[0]->get_center());
	real_t dydx = (center2.y - center.y) / (center2.x - center.x);
	Matrix S = Matrix::I(3);
	S.get(1, 0) = dydx;
	Matrix Sinv(3, 3);
	S.inverse(Sinv);
	REP(i, n)
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
	/*
	 *P(left);
	 *P(right);
	 *P(res);
	 */

	return Vec2D(res.get(0, 0), res.get(1, 0));
}

Matrix Panorama::shift_to_line(const vector<Vec2D>& ptr, const Vec2D& line) {
	m_assert(ptr.size() >= 9);

}
