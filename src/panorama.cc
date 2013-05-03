// File: panorama.cc
// Date: Fri May 03 18:01:21 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <fstream>
#include "panorama.hh"
#include "matcher.hh"
#include "utils.hh"
#include "cylinder.hh"
#include "keypoint.hh"
#include "warper.hh"
#include "transformer.hh"
using namespace std;

imgptr Panorama::get()
{ return get_trans(); }

imgptr Panorama::get_trans() {
	int n = imgs.size();
	vector<Matrix> mat;
	vector<pair<Vec2D, Vec2D>> corners;
	if (PANO) {
		Panorama::cal_best_matrix_pano(imgs, mat, corners);
		straighten_simple(mat, imgs);
	} else
		Panorama::cal_best_matrix(imgs, mat, corners);

	if (CIRCLE) { // remove the extra
		mat.pop_back();
		imgs.pop_back();
		corners.pop_back();
	}

	Vec2D min(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
		  max = min * (-1);
	for (auto &i : corners) {
		max.update_max(i.first);
		min.update_min(i.second);
	}

	Vec2D diff = max - min,
		  offset = min * (-1);
	Coor size = toCoor(diff);
	PP("size: ", size);
	PP("offset", offset);

	// inverse
	for_each(mat.begin(), mat.end(),
		[](Matrix & m) {
			Matrix inv(m.w, m.h);
			bool ok = m.inverse(inv);
			m_assert(ok);
			m = move(inv);
		});

	imgptr ret(new Img(size.x, size.y));
	ret->fill(Color::WHITE);

	// blending
	HWTimer timer;
#pragma omp parallel for schedule(dynamic)
	REP(i, ret->h)
		REP(j, ret->w) {
		Vec2D final = (Vec2D(j, i) - offset);
		vector<pair<Color, real_t>> blender;
		REP(k, n) {
			pair<Vec2D, Vec2D>& nowcorner = corners[k];
			if (!between(final.y, nowcorner.second.y, nowcorner.first.y) ||
					!between(final.x, nowcorner.second.x, nowcorner.first.x))
				continue;
			Vec2D old = TransFormer::cal_project(mat[k], final);
			if (between(old.x, 0, imgs[k]->w) && between(old.y, 0, imgs[k]->h)) {
				if (imgs[k]->is_black_edge(old.y, old.x)) continue;
				blender.push_back({imgs[k]->get_pixel(old.y, old.x),
						std::max(imgs[k]->w / 2 - abs(imgs[k]->get_center().x - old.x), 1e-1)});
			}
		}
		int ncolor = blender.size();
		if (!ncolor) continue;
		Color finalc;
		real_t sumweight = 0;
		/*
		 *for (auto &c : blender) {
		 *    finalc = finalc + c.first * c.second;
		 *    sumweight += c.second;
		 *}
		 *m_assert(fabs(sumweight) > EPS);
		 *finalc = finalc * (1.0 / sumweight);
		 */
		for (auto &c : blender)
			finalc = finalc + c.first;
		finalc = finalc * (1.0 / blender.size());
		ret->set_pixel(i, j, finalc);
	}
	print_debug("blend time: %lf secs\n", timer.get_sec());
	return ret;
}

Matrix Panorama::get_transform(const vector<Feature>& feat1, const vector<Feature>& feat2) {
	Matcher match(feat1, feat2);		// this is not efficient
	auto ret = match.match();
	TransFormer transf(ret, feat1, feat2);
	return move(transf.get_transform());
}

vector<Feature> Panorama::get_feature(imgptr & ptr) {
	ScaleSpace ss(ptr, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.work();
	return move(ex.features);
}

void Panorama::straighten_simple(vector<Matrix>& mat, const vector<imgptr>& imgs) {
	int n = imgs.size();
	Vec2D center2 = imgs[n - 1]->get_center();
	center2 = TransFormer::cal_project(mat[n - 1], center2);
	Vec2D center1 = TransFormer::cal_project(mat[0], imgs[0]->get_center());
	real_t dydx = (center2.y - center1.y) / (center2.x - center1.x);
	Matrix S = Matrix::I(3);
	S.get(1, 0) = dydx;
	Matrix Sinv(3, 3);
	S.inverse(Sinv);
	REP(i, n) mat[i] = Sinv.prod(mat[i]);
}

vector<pair<Vec2D, Vec2D>> Panorama::cal_size(const vector<Matrix>& mat, const vector<imgptr>& imgs) {
	int n = imgs.size();
	vector<pair<Vec2D, Vec2D>> ret;

	REPL(i, 0, n) {
		Vec2D min(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
			  max = min * (-1);
	    Vec2D corner[4] = { Vec2D(0, 0), Vec2D(imgs[i]->w, 0),
					        Vec2D(0, imgs[i]->h), Vec2D(imgs[i]->w, imgs[i]->h)};
	    for (auto &v : corner) {
	        Vec2D newcorner = TransFormer::cal_project(mat[i], v);
	        min.update_min(Vec2D(floor(newcorner.x), floor(newcorner.y)));
	        max.update_max(Vec2D(ceil(newcorner.x), ceil(newcorner.y)));
	    }
		ret.push_back({max, min});
	}
	return move(ret);
}

#define prepare() \
	m_assert(mat.size() == 0);\
	int n = imgs.size(), mid = n >> 1;\
	vector<vector<Feature>> feats;\
	feats.resize(n);\
	Matrix I = Matrix::I(3);\
	mat.resize(n, I);\
	HWTimer timer

void Panorama::cal_best_matrix_pano(vector<imgptr>& imgs, vector<Matrix>& mat, vector<pair<Vec2D, Vec2D>>& corners) {;
	prepare();
#pragma omp parallel for schedule(dynamic)
	REP(k, n)
		feats[k] = Panorama::get_feature(imgs[k]);
	print_debug("feature takes %lf secs in total\n", timer.get_sec());

	vector<MatchData> matches;
	REP(k, n - 1) {
		Matcher match(feats[k], feats[k + 1]);
		matches.push_back(match.match());
	}

	Matcher match(feats[n - 1], feats[0]);
	auto matched = match.match();
	print_debug("match time: %lf secs\n", timer.get_sec());

	// judge circle
	if ((real_t)matched.size() * 2 / (feats[0].size() + feats[n - 1].size()) > CONNECTED_THRES) {
		P("detect circle");
		CIRCLE = true;
		imgs.push_back(imgs[0]);
		mat.push_back(I);
		feats.push_back(feats[0]);
		matches.push_back(matched);
		n ++, mid = n >> 1;
	}
	vector<Matrix> bestmat;

	real_t minslope = numeric_limits<real_t>::max();
	real_t bestfactor = 0;

	timer.reset();
	int start = mid, end = n, len = end - start;
	if (len > 1) {
		real_t newfactor = 1;
		real_t slope = Panorama::update_h_factor(newfactor, minslope, bestfactor, bestmat, imgs, feats, matches);
		real_t centerx1 = imgs[mid]->get_center().x,
			   centerx2 = TransFormer::cal_project(bestmat[0], imgs[mid + 1]->get_center()).x;
		real_t order = (centerx2 > centerx1 ? 1 : -1);
		REP(k, 3) {
			if (fabs(slope) < SLOPE_PLAIN) break;
			newfactor += (slope < 0 ? order : -order) / (5 * pow(2, k));
			slope = Panorama::update_h_factor(newfactor, minslope, bestfactor, bestmat, imgs, feats, matches);
		}
	} else
		bestfactor = 1;

	Warper warper(bestfactor);
	REP(k, n) warper.warp(imgs[k], feats[k]);

	REPL(k, mid + 1, n) mat[k] = move(bestmat[k - mid - 1]);
	REPD(i, mid - 1, 0)
		mat[i] = TransFormer(matches[i].reverse(), feats[i + 1], feats[i]).get_transform();
		// mat[i] = Panorama::get_transform(feats[i + 1], feats[i]);

	REPD(i, mid - 2, 0) mat[i] = mat[i + 1].prod(mat[i]);
	print_debug("transform time: %lf secs \n", timer.get_sec());
	corners = cal_size(mat, imgs);
}

void Panorama::cal_best_matrix(vector<imgptr>& imgs, vector<Matrix>& mat, vector<pair<Vec2D, Vec2D>>& corners) {
	prepare();

	if (!TRANS) {
		Warper warper(1);
		REP(k, n) warper.warp(imgs[k], feats[k]);
	}

#pragma omp parallel for schedule(dynamic)
	REP(k, n)
		feats[k] = Panorama::get_feature(imgs[k]);
	print_debug("feature takes %lf secs in total\n", timer.get_sec());

#pragma omp parallel for schedule(dynamic)
	REPL(k, mid + 1, n)
		mat[k] = Panorama::get_transform(feats[k - 1], feats[k]);
	REPL(k, mid + 2, n) mat[k] = mat[k - 1].prod(mat[k]);
#pragma omp parallel for schedule(dynamic)
	REPD(k, mid - 1, 0)
		mat[k] = Panorama::get_transform(feats[k + 1], feats[k]);
	REPD(k, mid - 2, 0) mat[k] = mat[k + 1].prod(mat[k]);
	print_debug("match and transform time: %lf secs \n", timer.get_sec());
	corners = cal_size(mat, imgs);
}
#undef prepare

real_t Panorama::update_h_factor(real_t nowfactor,
								real_t & minslope,
								real_t & bestfactor,
								vector<Matrix>& mat,
								const vector<imgptr>& imgs,
								const vector<vector<Feature>>& feats,
								const vector<MatchData>& matches) {
	int n = imgs.size(), mid = n >> 1;
	int start = mid, end = n, len = end - start;

	Warper warper(nowfactor);

	vector<imgptr> nowimgs;
	vector<vector<Feature>> nowfeats;

	REPL(k, start, end) {
		nowimgs.push_back(imgs[k]);
		nowfeats.push_back(feats[k]);
	}			// nowfeats[0] == feats[mid]

#pragma omp parallel for schedule(dynamic)
	REP(k, len)
		warper.warp(nowimgs[k], nowfeats[k]);

	vector<Matrix> nowmat;		// size = len - 1
	REPL(k, 1, len)
		nowmat.push_back(TransFormer(matches[k - 1 + mid], nowfeats[k - 1], nowfeats[k]).get_transform());
		// nowmat[k - 1] = TransFormer(matches[k - 1 + mid], nowfeats[k - 1], nowfeats[k]).get_transform();
		//nowmat.push_back(Panorama::get_transform(nowfeats[k - 1], nowfeats[k]));
	REPL(k, 1, len - 1)
		nowmat[k] = nowmat[k - 1].prod(nowmat[k]);

	Vec2D center2 = TransFormer::cal_project(nowmat[len - 2],
											nowimgs[len - 2]->get_center()),
		  center1 = nowimgs[0]->get_center();
	real_t slope = (center2.y - center1.y) / (center2.x - center1.x);
	if (update_min(minslope, fabs(slope))) {
		bestfactor = nowfactor;
		mat = nowmat;
	}
	return slope;
}

/*
 *
 *Matrix Panorama::shift_to_line(const vector<Vec2D>& ptr, const Vec2D& line) {
 *    int n = ptr.size();
 *    m_assert(n >= 4);
 *    Matrix left(4, 2 * n);
 *    Matrix right(1, 2 * n);
 *    REP(k, n) {
 *        auto & nowp = ptr[k];
 *        real_t targetx = (nowp.x - (line.y - nowp.y) / line.x) / 2;
 *        real_t targety = line.x * targetx + line.y;
 *        left.get(2 * k, 0) = nowp.x;
 *        left.get(2 * k, 1) = nowp.y;
 *        left.get(2 * k, 2) = left.get(2 * k, 3) = 0;
 *        right.get(2 * k, 0) = targetx;
 *
 *        left.get(2 * k + 1, 0) = left.get(2 * k + 1, 1) = 0;
 *        left.get(2 * k + 1, 2) = nowp.x;
 *        left.get(2 * k + 1, 3) = nowp.y;
 *        right.get(2 * k + 1, 0) = targety;
 *    }
 *    Matrix res(1, 4);
 *    if (!left.solve_overdetermined(res, right)) {
 *        cout << "line_fit solve failed" << endl;
 *        return move(res);
 *    }
 *    Matrix ret(3, 3);
 *    ret.get(0, 0) = res.get(0, 0);
 *    ret.get(0, 1) = res.get(1, 0);
 *    ret.get(1, 0) = res.get(2, 0);
 *    ret.get(1, 1) = res.get(3, 0);
 *    ret.get(2, 2) = 1;
 *    for (auto &i : ptr) {
 *        Vec2D project = TransFormer::cal_project(ret, i);
 *        cout << project << " ==?" << (line.x * project.x + line.y) << endl;
 *    }
 *    return move(ret);
 *}
 *
 */

/*
 *void Panorama::straighten(vector<Matrix>& mat) const {
 *    int n = mat.size();
 *
 *    vector<Vec2D> centers;
 *    REP(k, n)
 *        centers.push_back(TransFormer::cal_project(mat[k], imgs[k]->get_center()));
 *    Vec2D kb = Panorama::line_fit(centers);
 *    P(kb);
 *    if (fabs(kb.x) < 1e-3) return;		// already done
 *    Matrix shift = Panorama::shift_to_line(centers, kb);
 *    P(shift);
 *    for (auto& i : mat) i = shift.prod(i);
 *}
 *
 *Vec2D Panorama::line_fit(const std::vector<Vec2D>& pts) {
 *    int n = pts.size();
 *
 *    Matrix left(2, n);
 *    Matrix right(1, n);
 *    REP(k, n) {
 *        left.get(k, 0) = pts[k].x, left.get(k, 1) = 1;
 *        right.get(k, 0) = pts[k].y;
 *    }
 *
 *    Matrix res(0, 0);
 *    if (!left.solve_overdetermined(res, right)) {
 *        cout << "line_fit solve failed" << endl;
 *        return Vec2D(0, 0);
 *    }
 *    return Vec2D(res.get(0, 0), res.get(1, 0));
 *}
 *
 */
