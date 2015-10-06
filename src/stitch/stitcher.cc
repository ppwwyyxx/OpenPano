// File: stitcher.cc
// Date: Sun Sep 22 12:54:18 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "stitcher.hh"

#include <fstream>
#include <algorithm>

#include "feature/matcher.hh"
#include "cylinder.hh"
#include "warper.hh"
#include "transformer.hh"

#include "lib/timer.hh"
#include "lib/imgproc.hh"
#include "blender.hh"
using namespace std;

Mat32f Stitcher::build() {
	calc_feature();
	calc_transform();	// calculate pairwise transform
	bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
	if (TRANS)
		bundle.proj_method = ConnectedImages::ProjectionMethod::cylindrical;
	print_debug("Using projection method: %d\n", bundle.proj_method);
		//bundle.proj_method = ConnectedImages::ProjectionMethod::spherical;
	bundle.update_proj_range(imgs);

	return blend();
}

Mat32f Stitcher::blend() {
	int n = imgs.size();

	int refw = imgs[bundle.identity_idx].width(),
			refh = imgs[bundle.identity_idx].height();
	auto homo2proj = bundle.get_homo2proj();
	auto proj2homo = bundle.get_proj2homo();

	Vec2D id_img_range = homo2proj(Vec(1, 1, 1)) - homo2proj(Vec(0, 0, 1));
	id_img_range.x *= refw, id_img_range.y *= refh;
	cout << "id_img_range" << id_img_range << endl;
	cout << "projmin:" << bundle.proj_min << "projmax" << bundle.proj_max << endl;

	Vec2D proj_min = bundle.proj_min;
	real_t x_len = bundle.proj_max.x - proj_min.x,
		   y_len = bundle.proj_max.y - proj_min.y,
		   x_per_pixel = id_img_range.x / refw,
		   y_per_pixel = id_img_range.y / refh,
		   target_width = x_len / x_per_pixel,
		   target_height = y_len / y_per_pixel;

	Coor size(target_width, target_height);
	cout << "Final Image Size: " << size << endl;

	auto scale_coor_to_img_coor = [&](Vec2D v) {
		v = v - proj_min;
		v.x /= x_per_pixel, v.y /= y_per_pixel;
		return Coor(v.x, v.y);
	};

	// blending
	GuardedTimer tm("Blending");
	Mat32f ret(size.y, size.x, 3);
	fill(ret, Color::NO);

	LinearBlender blender;
	REP(k, n) {
		auto& cur_img = bundle.component[k];
		Coor top_left = scale_coor_to_img_coor(bundle.proj_ranges[k].min);
		Coor bottom_right = scale_coor_to_img_coor(bundle.proj_ranges[k].max);
		Coor diff = bottom_right - top_left;
		int w = diff.x, h = diff.y;
		Mat<Vec2D> orig_pos(h, w, 1);

		REP(i, h) REP(j, w) {
			Vec2D c((j + top_left.x) * x_per_pixel + proj_min.x, (i + top_left.y) * y_per_pixel + proj_min.y);
			Vec homo = proj2homo(Vec2D(c.x / imgs[k].width(),
								c.y / imgs[k].height()));
			homo.x *= refw, homo.y *= refh;
			Vec2D& p = (orig_pos.at(i, j)
					= cur_img.homo_inv.trans_normalize(homo));
			if (!p.isNaN() && (p.x < 0 || p.x >= imgs[k].width() || p.y < 0 || p.y >= imgs[k].height()))
				p = Vec2D::NaN();
		}
		blender.add_image(top_left, orig_pos, imgs[k]);
	}
	blender.run(ret);
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

void Stitcher::straighten_simple() {
	int n = imgs.size();
	// wrong
	Vec2D center2 = bundle.component[n - 1].homo.trans2d(0.5, 0.5);
	Vec2D center1 = bundle.component[0].homo.trans2d(0.5, 0.5);
	float dydx = (center2.y - center1.y) / (center2.x - center1.x);
	Matrix S = Matrix::I(3);
	S.at(1, 0) = dydx;
	Matrix Sinv(3, 3);
	bool succ = S.inverse(Sinv);
	m_assert(succ);
	REP(i, n) bundle.component[i].homo = Sinv.prod(bundle.component[i].homo);
}

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
		calc_matrix_pano();
		//straighten_simple();

		if (circle_detected) { // remove the extra
			bundle.component.pop_back();
			imgs.pop_back();
		}
	} else {
		calc_matrix_simple();
	}
	print_debug("match & transform takes %lf secs\n", timer.duration());

	bundle.calc_inverse_homo();
}

void Stitcher::calc_matrix_pano() {;
	int n = imgs.size(), mid = n >> 1;
	bundle.component.resize(n);
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
	bundle.identity_idx = mid;
	vector<Homography> bestmat;
	auto& mid_img = imgs[mid];

	float minslope = numeric_limits<float>::max();
	float bestfactor = 0;

	GuardedTimer tm("transform");
	int start = mid, end = n, len = end - start;
	if (len > 1) {
		float newfactor = 1;
		// XXX: ugly
		float slope = update_h_factor(newfactor, minslope, bestfactor, bestmat, matches);
		if (bestmat.empty()) {
			cout << "Failed to find hfactor" << endl;
			exit(1);
		}
		float centerx1 = mid_img.width() / 2,
					centerx2 = bestmat[0].trans2d(imgs[mid+1].width() / 2, imgs[mid+1].height() / 2).x;
		float order = (centerx2 > centerx1 ? 1 : -1);
		REP(k, 3) {
			if (fabs(slope) < SLOPE_PLAIN) break;
			newfactor += (slope < 0 ? order : -order) / (5 * pow(2, k));
			slope = Stitcher::update_h_factor(newfactor, minslope, bestfactor, bestmat, matches);
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
	/*
	 *vector<float> fs;
	 *REPL(k, 0, n - 1) {
	 *  auto m = get_transform(k, k+1);
	 *  auto f = TransFormer::get_focal_from_matrix(m);
	 *  fs.emplace_back(f);
	 *}
	 *REPL(k, 1, n) {
	 *  auto m = get_transform(k, k-1);
	 *  auto f = TransFormer::get_focal_from_matrix(m);
	 *  fs.emplace_back(f);
	 *}
	 *sort(fs.begin(), fs.end());
	 *print_debug("focal: %f", fs[fs.size() / 2]);
	 */
	// accumulate the transformations
	REPL(k, mid + 2, n)
		comp[k].homo = Homography(
				comp[k - 1].homo.prod(comp[k].homo));
	REPD(k, mid - 2, 0)
		comp[k].homo = Homography(
					comp[k + 1].homo.prod(comp[k].homo));
	// then, comp[k]: from k to identity
}

// XXX ugly hack
float Stitcher::update_h_factor(float nowfactor,
		float & minslope,
		float & bestfactor,
		vector<Homography>& mat,
		const vector<MatchData>& matches) {
	const int n = imgs.size(), mid = n >> 1;
	const int start = mid, end = n, len = end - start;

	vector<Mat32f> nowimgs;
	vector<vector<Descriptor>> nowfeats;
	REPL(k, start, end) {
		nowimgs.push_back(imgs[k].clone());
		nowfeats.push_back(feats[k]);
	}			// nowfeats[0] == feats[mid]

	Warper warper(nowfactor);
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

	REPL(k, 1, len - 1)
		nowmat[k] = nowmat[k - 1].prod(nowmat[k]);	// transform to nowimgs[0] == imgs[mid]

	Vec2D center2 = nowmat.back().trans2d(nowimgs.back().width() / 2, nowimgs.back().height() / 2);
	Vec2D	center1(nowimgs[0].width() / 2, nowimgs[0].height() / 2);
	const float slope = (center2.y - center1.y) / (center2.x - center1.x);
	print_debug("slope: %lf\n", slope);
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
