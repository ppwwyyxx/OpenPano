// File: stitcher.cc
// Date: Sun Sep 22 12:54:18 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>
// XXX: I know it is somehow ugly


#include "stitcher.hh"

#include <fstream>
#include <algorithm>

#include "feature/matcher.hh"
#include "cylinder.hh"
#include "warper.hh"
#include "transformer.hh"

#include "lib/timer.hh"
#include "lib/imgproc.hh"
using namespace std;

Mat32f Stitcher::build() {
	calc_feature();
	calc_transform();	// calculate pairwise transform
	int n = imgs.size();

	// get size
	cal_size();
	Vec2D min(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
		  max = min * (-1);
	for (auto &i : corners) {
		max.update_max(i.first);
		min.update_min(i.second);
	}

	Vec2D diff = max - min,
		  offset = min * (-1);
	Coor size = toCoor(diff);
	cout << "size: " << size << endl;

	auto& comp = bundle.component;

	Mat32f ret(size.y, size.x, 3);
	fill(ret, Color::NO);

	// blending
	GuardedTimer tm("Blending");
#pragma omp parallel for schedule(dynamic)
	REP(i, ret.height())
		REP(j, ret.width()) {
		Vec2D final = (Vec2D(j, i) - offset);
		vector<pair<Color, float>> blender;
		REP(k, n) {
			pair<Vec2D, Vec2D>& nowcorner = corners[k];
			if (!between(final.y, nowcorner.second.y, nowcorner.first.y) ||
					!between(final.x, nowcorner.second.x, nowcorner.first.x))
				continue;
			Vec2D old = TransFormer::calc_project(comp[k].homo_inv, final);
			if (!is_edge_color(imgs[k], old.y, old.x)) {
				blender.push_back({interpolate(imgs[k], old.y, old.x),
						std::max(imgs[k].width() / 2 - abs(imgs[k].width() / 2 - old.x), .1f)});
			}
		}
		int ncolor = blender.size();
		if (!ncolor) continue;
		Color finalc;

		float sumweight = 0;
		for (auto &c : blender) {
			finalc = finalc + c.first * c.second;
			sumweight += c.second;
		}
		m_assert(fabs(sumweight) > EPS);
		finalc = finalc * (1.0 / sumweight);
		/*
		 *for (auto &c : blender) finalc = finalc + c.first; // noblend
		 *finalc = finalc * (1.0 / blender.size());
		 */

		float* p = ret.ptr(i, j);
		finalc.write_to(p);
	}
	return ret;
}

Homography Stitcher::get_transform(int f1, int f2) const {
	Matcher match(feats[f1], feats[f2]);		// this is not efficient
	auto ret = match.match();
	TransFormer transf(ret, feats[f1], feats[f2]);
	Homography r;
	bool succ = transf.get_transform(&r);
	if (not succ)
		error_exit(ssprintf("Image %d & %d doesn't match.", f1, f2));
	return r;
}

/*
 *void Stitcher::straighten_simple() {
 *  int n = imgs.size();
 *  Vec2D center2(imgs[n - 1].width() / 2, imgs[n-1].height() / 2);
 *  center2 = TransFormer::calc_project(mat[n - 1], center2);
 *  Vec2D center1 = TransFormer::calc_project(mat[0], Vec2D(imgs[0].width() / 2, imgs[0].height() / 2));
 *  float dydx = (center2.y - center1.y) / (center2.x - center1.x);
 *  Matrix S = Matrix::I(3);
 *  S.at(1, 0) = dydx;
 *  Matrix Sinv(3, 3);
 *  bool succ = S.inverse(Sinv);
 *  m_assert(succ);
 *  REP(i, n) mat[i] = Sinv.prod(mat[i]);
 *}
 */

void Stitcher::calc_feature() {
	GuardedTimer tm("calc_feature");
	int n = imgs.size();
	// detect feature
	feats.resize(n);
#pragma omp parallel for schedule(dynamic)
	REP(k, n)
		feats[k] = detect_SIFT(imgs[k]);
}

void Stitcher::calc_transform() {
	Timer timer;
	if (PANO) {
		cal_best_matrix_pano();
		//straighten_simple();
	} else {
		calc_matrix_simple();
		print_debug("match & transform takes %lf secs\n", timer.duration());
	}

	if (circle_detected) { // remove the extra
		bundle.component.pop_back();
		imgs.pop_back();
	}
	bundle.calc_inverse_homo();
}

void Stitcher::cal_size() {
	int n = imgs.size();

	REPL(i, 0, n) {
		Vec2D min(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
			  max = min * (-1);
	    Vec2D corner[4] = { Vec2D(0, 0), Vec2D(imgs[i].width(), 0),
					        Vec2D(0, imgs[i].height()), Vec2D(imgs[i].width(), imgs[i].height())};
	    for (auto &v : corner) {
	        Vec2D newcorner = TransFormer::calc_project(bundle.component[i].homo, v);
	        min.update_min(Vec2D(floor(newcorner.x), floor(newcorner.y)));
	        max.update_max(Vec2D(ceil(newcorner.x), ceil(newcorner.y)));
	    }
		corners.push_back({max, min});
	}
	return;
}

void Stitcher::cal_best_matrix_pano() {;
	int n = imgs.size(), mid = n >> 1;
	bundle.component.resize(n);
	bundle.identity_idx = mid;
	REP(i, n) bundle.component[i].homo = Matrix::I(3);

	Timer timer;
	vector<MatchData> matches;
	matches.resize(n == 2 ? 1 : n);
	REP(k, n == 2 ? 1 : n) {
		Matcher matcher(feats[k], feats[(k + 1) % n]);
		matches[k] = matcher.match();
	}
	print_debug("match time: %lf secs\n", timer.duration());

	if (n > 2) {
		// head and tail
		auto last_match = matches.back();
		// test whether two image really matches each other
		if ((float)last_match.size() * 2 / (feats[0].size() + feats[n - 1].size()) > CONNECTED_THRES) {
			print_debug("detect circle\n");
			circle_detected = true;
			imgs.push_back(imgs[0].clone());
			bundle.component.emplace_back(ImageComponent{Homography::I()});
			feats.push_back(feats[0]);
			n ++, mid = n >> 1;
		} else {
			matches.pop_back();
		}
	}
	vector<Homography> bestmat;

	float minslope = numeric_limits<float>::max();
	float bestfactor = 0;

	GuardedTimer tm("transform");
	int start = mid, end = n, len = end - start;
	if (len > 1) {
		float newfactor = 1;
		float slope = Stitcher::update_h_factor(newfactor, minslope, bestfactor, bestmat, imgs, feats, matches);
		if (bestmat.empty()) {
			cout << "Failed to find hfactor" << endl;
			exit(1);
		}
		float centerx1 = imgs[mid].width() / 2,
					centerx2 = TransFormer::calc_project(
							bestmat[0], Vec2D(imgs[mid + 1].width() / 2, imgs[mid + 1].height() / 2)).x;
		float order = (centerx2 > centerx1 ? 1 : -1);
		REP(k, 3) {
			if (fabs(slope) < SLOPE_PLAIN) break;
			newfactor += (slope < 0 ? order : -order) / (5 * pow(2, k));
			slope = Stitcher::update_h_factor(newfactor, minslope, bestfactor, bestmat, imgs, feats, matches);
		}
	} else
		bestfactor = 1;
	print_debug("Best hfactor: %lf\n", bestfactor);
	Warper warper(bestfactor);
	REP(k, n) warper.warp(imgs[k], feats[k]);

	REPL(k, mid + 1, n) bundle.component[k].homo = move(bestmat[k - mid - 1]);
	// TODO can we use inverse transform directly?
	REPD(i, mid - 1, 0) {
		matches[i].reverse();
		bool succ = TransFormer(
				matches[i],
				feats[i + 1], feats[i]).get_transform(&bundle.component[i].homo);
		if (not succ) {
			cerr << "The two image doesn't match. Failed" << endl;
			exit(1);
		}
	}

	REPD(i, mid - 2, 0)
		bundle.component[i].homo = Homography(
				bundle.component[i + 1].homo.prod(bundle.component[i].homo));
}

void Stitcher::calc_matrix_simple() {
	int n = imgs.size(), mid = n >> 1;
	bundle.identity_idx = mid;
	bundle.component.resize(n);
	bundle.component[mid] = ImageComponent{Homography::I()};

	// when not translation, do a simple-guess warping
	if (!TRANS) {
		Warper warper(1);
		REP(k, n) warper.warp(imgs[k], feats[k]);
	}

	auto& comp = bundle.component;

	// transform w.r.t the identity one
#pragma omp parallel for schedule(dynamic)
	REP(k, n) {
		if (k >= mid + 1)
			// get match and transform
			comp[k] = ImageComponent{get_transform(k - 1, k)};	// from k to k-1
		else if (k <= mid - 1)
			comp[k] = ImageComponent{get_transform(k + 1, k)};	// from k to k+1
	}
	// accumulate the transformations
	REPL(k, mid + 2, n)
		comp[k].homo = Homography(
				comp[k - 1].homo.prod(comp[k].homo));
	REPD(k, mid - 2, 0)
		comp[k].homo = Homography(
				comp[k + 1].homo.prod(comp[k].homo));
	// then, comp[k]: from k to identity
}
#undef prepare

float Stitcher::update_h_factor(float nowfactor,
		float & minslope,
		float & bestfactor,
		vector<Homography>& mat,
		const vector<Mat32f>& imgs,
		const vector<vector<Descriptor>>& feats,
		const vector<MatchData>& matches) {

	const int n = imgs.size(), mid = n >> 1;
	const int start = mid, end = n, len = end - start;

	Warper warper(nowfactor);

	vector<Mat32f> nowimgs;
	vector<vector<Descriptor>> nowfeats;

	REPL(k, start, end) {
		nowimgs.push_back(imgs[k].clone());
		nowfeats.push_back(feats[k]);
	}			// nowfeats[0] == feats[mid]

#pragma omp parallel for schedule(dynamic)
	REP(k, len)
		warper.warp(nowimgs[k], nowfeats[k]);

	vector<Homography> nowmat;		// size = len - 1
	REPL(k, 1, len) {
		nowmat.emplace_back();
		bool succ = TransFormer(matches[k - 1 + mid], nowfeats[k - 1],
			nowfeats[k]).get_transform(&nowmat.back());
		if (not succ) {
			cerr << "The two image doesn't match. Failed" << endl;
			exit(1);
		}
	}
	// nowmat[k - 1] = TransFormer(matches[k - 1 + mid], nowfeats[k - 1], nowfeats[k]).get_transform();
	//nowmat.push_back(Stitcher::get_transform(nowfeats[k - 1], nowfeats[k]));
	REPL(k, 1, len - 1)
		nowmat[k] = nowmat[k - 1].prod(nowmat[k]);

	Vec2D center2 = TransFormer::calc_project(
			nowmat[len - 2],
			Vec2D(nowimgs[len - 2].width() / 2, nowimgs[len - 2].height() / 2)),
				center1(nowimgs[0].width() / 2, nowimgs[0].height() / 2);
	const float slope = (center2.y - center1.y) / (center2.x - center1.x);
	if (update_min(minslope, fabs(slope))) {
		bestfactor = nowfactor;
		mat = move(nowmat);
	}
	return slope;
}

/*
 *
 *Matrix Stitcher::shift_to_line(const vector<Vec2D>& ptr, const Vec2D& line) {
 *    int n = ptr.size();
 *    m_assert(n >= 4);
 *    Matrix left(4, 2 * n);
 *    Matrix right(1, 2 * n);
 *    REP(k, n) {
 *        auto & nowp = ptr[k];
 *        float targetx = (nowp.x - (line.y - nowp.y) / line.x) / 2;
 *        float targety = line.x * targetx + line.y;
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
 *void Stitcher::straighten(vector<Matrix>& mat) const {
 *    int n = mat.size();
 *
 *    vector<Vec2D> centers;
 *    REP(k, n)
 *        centers.push_back(TransFormer::cal_project(mat[k], imgs[k]->get_center()));
 *    Vec2D kb = Stitcher::line_fit(centers);
 *    P(kb);
 *    if (fabs(kb.x) < 1e-3) return;		// already done
 *    Matrix shift = Stitcher::shift_to_line(centers, kb);
 *    P(shift);
 *    for (auto& i : mat) i = shift.prod(i);
 *}
 *
 *Vec2D Stitcher::line_fit(const std::vector<Vec2D>& pts) {
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
