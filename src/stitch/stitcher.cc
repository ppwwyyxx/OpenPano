// File: stitcher.cc
// Date: Sun Sep 22 12:54:18 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "stitcher.hh"

#include <limits>
#include <string>
#include <cmath>
#include <queue>

#include "feature/matcher.hh"
#include "lib/imgproc.hh"
#include "lib/timer.hh"
#include "blender.hh"
#include "match_info.hh"
#include "transform_estimate.hh"
#include "camera_estimator.hh"
#include "warp.hh"
using namespace std;
using namespace feature;
using namespace projector;
using namespace config;

namespace stitch {

// use in development
const static bool DEBUG_OUT = false;
const static char* MATCHINFO_DUMP = "log/matchinfo.txt";

Mat32f Stitcher::build() {
	calc_feature();
	// TODO choose a better starting point by MST use centrality
	if (CYLINDER) {
		assign_center();
		build_bundle_warp();
		bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
	} else {
		if (TRANS)
			assume_linear_pairwise();
		else
			pairwise_match();
		feats.clear(); feats.shrink_to_fit();	// free memory for feature
		//load_matchinfo(MATCHINFO_DUMP);
		if (DEBUG_OUT) {
			draw_matchinfo();
			dump_matchinfo(MATCHINFO_DUMP);
		}
		assign_center();

		if (ESTIMATE_CAMERA)
			estimate_camera();
		else
			build_bundle_linear_simple();		// naive mode
		// TODO automatically determine projection method
		if (ESTIMATE_CAMERA)
			bundle.proj_method = ConnectedImages::ProjectionMethod::cylindrical;
		else
			bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
	}
	print_debug("Using projection method: %d\n", bundle.proj_method);
	bundle.update_proj_range();
	return blend();
}

void Stitcher::calc_feature() {
	GuardedTimer tm("calc_feature()");
	int n = imgs.size();
	feats.resize(n);
	// detect feature
#pragma omp parallel for schedule(dynamic)
	REP(k, n) {
		feats[k] = feature_det->detect_feature(imgs[k]);
		if (feats[k].size() == 0)	// TODO delete the image
			error_exit("Cannot find feature in this image!");
		print_debug("Image %d has %lu features\n", k, feats[k].size());
	}
}

void Stitcher::pairwise_match() {
	GuardedTimer tm("pairwise_match() with transform");

	PairWiseMatcher pwmatcher(feats);
	size_t n = imgs.size();
	pairwise_matches.resize(n);
	for (auto& k : pairwise_matches) k.resize(n);

	vector<pair<int, int>> tasks;
	REP(i, n) REPL(j, i + 1, n) tasks.emplace_back(i, j);

#pragma omp parallel for schedule(dynamic)
	REP(k, tasks.size()) {
		int i = tasks[k].first, j = tasks[k].second;
		//auto match = matcher(feats[i], feats[j]).match();	// slow
		auto match = pwmatcher.match(i, j);
		TransformEstimation transf(match, feats[i], feats[j], {imgs[i].width(), imgs[i].height()});	// from j to i
		MatchInfo info;
		bool succ = transf.get_transform(&info);
		if (not succ) {
			//print_debug("Only %d inlier from %d to %d\n", -(int)info.confidence, i, j);
			continue;
		}
		auto inv = info.homo.inverse(&succ);
		inv.mult(1.0 / inv[8]);	// more stable?
		if (not succ) continue;	// cannot inverse. mal-formed homography
		print_debug(
				"Connection between image %d and %d, ninliers=%lu, conf=%f\n",
				i, j, info.match.size(), info.confidence);
		// fill in pairwise matches
		pairwise_matches[i][j] = info;
		info.homo = inv;
		info.reverse();
		pairwise_matches[j][i] = move(info);
	}
}

void Stitcher::assume_linear_pairwise() {
	GuardedTimer tm("assume_linear_pairwise()");
	int n = imgs.size();
	PairWiseMatcher pwmatcher(feats);
#pragma omp parallel for schedule(dynamic)
	REP(i, n-1) {
		int next = (i + 1) % n;
		auto match = pwmatcher.match(i, next);
		TransformEstimation transf(match, feats[i], feats[next], {imgs[i].width(), imgs[i].height()});
		MatchInfo info;
		bool succ = transf.get_transform(&info);
		if (not succ)
			error_exit(ssprintf("Image %d and %d doesn't match.\n", i, next));
		auto inv = info.homo.inverse(&succ);
		if (not succ) // cannot inverse. mal-formed homography
			error_exit(ssprintf("Image %d and %d doesn't match.\n", i, next));
		print_debug("Match between image %d and %d, ninliers=%lu, conf=%f\n",
				i, next, info.match.size(), info.confidence);
		pairwise_matches[i][next] = info;
		info.homo = inv;
		info.reverse();
		pairwise_matches[next][i] = move(info);
	}
}

void Stitcher::assign_center() {
	// naively. when changing here, keep mid for CYLINDER
	bundle.identity_idx = imgs.size() >> 1;
	//bundle.identity_idx = 0;
}

void Stitcher::estimate_camera() {
	vector<Shape2D> shapes;
	for (auto& m: imgs)
		shapes.emplace_back(m.cols(), m.rows());
	CameraEstimator ce(pairwise_matches, shapes);
	auto cameras = ce.estimate();

	REP(i, imgs.size()) {
		bundle.component[i].homo_inv = cameras[i].K() * cameras[i].R;
		bundle.component[i].homo = cameras[i].Rinv() * cameras[i].K().inverse();
	}
}


void Stitcher::build_bundle_linear_simple() {
	// TODO bfs over pairwise to build bundle
	// assume pano pairwise
	int n = imgs.size(), mid = bundle.identity_idx;
	bundle.component[mid].homo = Homography::I();

	auto& comp = bundle.component;

	// accumulate the transformations
	if (mid + 1 < n) {
		comp[mid+1].homo = pairwise_matches[mid][mid+1].homo;
		REPL(k, mid + 2, n)
			comp[k].homo = comp[k - 1].homo * pairwise_matches[k-1][k].homo;
	}
	if (mid - 1 >= 0) {
		comp[mid-1].homo = pairwise_matches[mid][mid-1].homo;
		REPD(k, mid - 2, 0)
			comp[k].homo = comp[k + 1].homo * pairwise_matches[k+1][k].homo;
	}
	// then, comp[k]: from k to identity
	bundle.calc_inverse_homo();
}


void Stitcher::build_bundle_warp() {;
	GuardedTimer tm("build_bundle_warp()");
	int n = imgs.size(), mid = bundle.identity_idx;
	REP(i, n) bundle.component[i].homo = Homography::I();

	Timer timer;
	vector<MatchData> matches;		// matches[k]: k,k+1
	PairWiseMatcher pwmatcher(feats);
	matches.resize(n-1);
#pragma omp parallel for schedule(dynamic)
	REP(k, n - 1)
		matches[k] = pwmatcher.match(k, (k + 1) % n);
	print_debug("match time: %lf secs\n", timer.duration());

	vector<Homography> bestmat;

	float minslope = numeric_limits<float>::max();
	float bestfactor = 1;
	if (n - mid > 1) {
		float newfactor = 1;
		// XXX: ugly
		float slope = update_h_factor(newfactor, minslope, bestfactor, bestmat, matches);
		if (bestmat.empty())
			error_exit("Failed to find hfactor");
		float centerx1 = 0, centerx2 = bestmat[0].trans2d(0, 0).x;
		float order = (centerx2 > centerx1 ? 1 : -1);
		REP(k, 3) {
			if (fabs(slope) < SLOPE_PLAIN) break;
			newfactor += (slope < 0 ? order : -order) / (5 * pow(2, k));
			slope = Stitcher::update_h_factor(newfactor, minslope, bestfactor, bestmat, matches);
		}
	}
	print_debug("Best hfactor: %lf\n", bestfactor);
	CylinderWarper warper(bestfactor);
#pragma omp parallel for schedule(dynamic)
	REP(k, n) warper.warp(imgs[k], feats[k]);

	// accumulate
	REPL(k, mid + 1, n) bundle.component[k].homo = move(bestmat[k - mid - 1]);
#pragma omp parallel for schedule(dynamic)
	REPD(i, mid - 1, 0) {
		matches[i].reverse();
		MatchInfo info;
		bool succ = TransformEstimation(
				matches[i], feats[i + 1], feats[i], {imgs[i+1].width(), imgs[i+1].height()}).get_transform(&info);
		if (not succ)
			error_exit(ssprintf("Image %d and %d doesn't match. Failed", i, i+1));
		bundle.component[i].homo = info.homo;
	}
	REPD(i, mid - 2, 0)
		bundle.component[i].homo = bundle.component[i + 1].homo * bundle.component[i].homo;
	bundle.calc_inverse_homo();
}

float Stitcher::update_h_factor(float nowfactor,
		float & minslope,
		float & bestfactor,
		vector<Homography>& mat,
		const vector<MatchData>& matches) {
	const int n = imgs.size(), mid = bundle.identity_idx;
	const int start = mid, end = n, len = end - start;

	vector<Mat32f> nowimgs;
	vector<vector<Descriptor>> nowfeats;
	REPL(k, start, end) {
		nowimgs.push_back(imgs[k].clone());
		nowfeats.push_back(feats[k]);
	}			// nowfeats[0] == feats[mid]

	CylinderWarper warper(nowfactor);
#pragma omp parallel for schedule(dynamic)
	REP(k, len)
		warper.warp(nowimgs[k], nowfeats[k]);

	vector<Homography> nowmat;		// size = len - 1
	nowmat.resize(len - 1);
	bool failed = false;
#pragma omp parallel for schedule(dynamic)
	REPL(k, 1, len) {
		MatchInfo info;
		bool succ = TransformEstimation(matches[k - 1 + mid], nowfeats[k - 1],
				nowfeats[k], {nowimgs[k-1].width(), nowimgs[k-1].height()}).get_transform(&info);
		if (not succ)
			failed = true;
		//error_exit("The two image doesn't match. Failed");
		nowmat[k-1] = info.homo;
	}
	if (failed) return 0;

	REPL(k, 1, len - 1)
		nowmat[k] = nowmat[k - 1] * nowmat[k];	// transform to nowimgs[0] == imgs[mid]

	// check the slope of the result image
	Vec2D center2 = nowmat.back().trans2d(0, 0);
	const float slope = center2.y/ center2.x;
	print_debug("slope: %lf\n", slope);
	if (update_min(minslope, fabs(slope))) {
		bestfactor = nowfactor;
		mat = move(nowmat);
	}
	return slope;
}

Mat32f Stitcher::perspective_correction(const Mat32f& img) {
	int w = img.width(), h = img.height();
	int refw = imgs[bundle.identity_idx].width(),
			refh = imgs[bundle.identity_idx].height();
	auto homo2proj = bundle.get_homo2proj();
	Vec2D proj_min = bundle.proj_range.min;

	vector<Vec2D> corners;
	auto cur = &(bundle.component.front());
	auto to_ref_coor = [&](Vec2D v) {
		v.x *= cur->imgptr->width(), v.y *= cur->imgptr->height();
		Vec homo = cur->homo.trans(v);
		homo.x /= refw, homo.y /= refh;
		homo.x += 0.5 * homo.z, homo.y += 0.5 * homo.z;
		Vec2D t_corner = homo2proj(homo);
		t_corner.x *= refw, t_corner.y *= refh;
		t_corner = t_corner - proj_min;
		corners.push_back(t_corner);
	};
	to_ref_coor(Vec2D(-0.5, -0.5));
	to_ref_coor(Vec2D(-0.5, 0.5));
	cur = &(bundle.component.back());
	to_ref_coor(Vec2D(0.5, -0.5));
	to_ref_coor(Vec2D(0.5, 0.5));

	// stretch the four corner to rectangle
	vector<Vec2D> corners_std = {
		Vec2D(0, 0), Vec2D(0, h),
		Vec2D(w, 0), Vec2D(w, h)};
	Matrix m = getPerspectiveTransform(corners, corners_std);
	Homography inv(m);

	LinearBlender blender;
	blender.add_image(Coor(0,0), Coor(w,h), img, [=](Coor c) -> Vec2D {
		return inv.trans2d(Vec2D(c.x, c.y));
	});
	auto ret = Mat32f(h, w, 3);
	fill(ret, Color::NO);
	blender.run(ret);
	return ret;
}

Mat32f Stitcher::blend() {
	GuardedTimer tm("blend()");
	// it's hard to do coordinates.......
	int refw = imgs[bundle.identity_idx].width(),
		refh = imgs[bundle.identity_idx].height();
	auto homo2proj = bundle.get_homo2proj();
	auto proj2homo = bundle.get_proj2homo();

	Vec2D id_img_range = homo2proj(Vec(1, 1, 1)) - homo2proj(Vec(0, 0, 1));
	id_img_range.x *= refw, id_img_range.y *= refh;
	cout << "projmin:" << bundle.proj_range.min << "projmax" << bundle.proj_range.max << endl;

	Vec2D proj_min = bundle.proj_range.min;
	double x_len = bundle.proj_range.max.x - proj_min.x,
		   y_len = bundle.proj_range.max.y - proj_min.y,
			 // TODO this gives better aspect ratio. why?
		   x_per_pixel = id_img_range.x / (bundle.proj_method == ConnectedImages::ProjectionMethod::flat ? refw : refh),
		   y_per_pixel = id_img_range.y / refh,
		   target_width = x_len / x_per_pixel,
		   target_height = y_len / y_per_pixel;

	Coor size(target_width, target_height);
	print_debug("Final Image Size: (%d, %d)\n", size.x, size.y);

	auto scale_coor_to_img_coor = [&](Vec2D v) {
		v = v - proj_min;
		v.x /= x_per_pixel, v.y /= y_per_pixel;
		return Coor(v.x, v.y);
	};

	// blending
	Mat32f ret(size.y, size.x, 3);
	fill(ret, Color::NO);

	LinearBlender blender;
	REP(comp_idx, bundle.component.size()) {
		auto& cur = bundle.component[comp_idx];
		Coor top_left = scale_coor_to_img_coor(cur.range.min);
		Coor bottom_right = scale_coor_to_img_coor(cur.range.max);

		int imgw = cur.imgptr->width(),
				imgh = cur.imgptr->height();

		blender.add_image(top_left, bottom_right, *cur.imgptr, [=,&cur](Coor t) -> Vec2D {
			Vec2D c(t.x * x_per_pixel + proj_min.x,
							t.y * y_per_pixel + proj_min.y);
			Vec homo = proj2homo(Vec2D(c.x / refw, c.y / refh));
			if (not ESTIMATE_CAMERA)  {	// scale and offset is in camera intrinsic
				homo.x -= 0.5 * homo.z, homo.y -= 0.5 * homo.z;	// shift center for homography
				homo.x *= refw, homo.y *= refh;
			}
			Vec2D orig = cur.homo_inv.trans_normalize(homo);
			if (not ESTIMATE_CAMERA)
				orig = orig + Vec2D(imgw/2, imgh/2);
			return orig;
		});
	}
	//if (DEBUG_OUT) blender.debug_run(size.x, size.y);
	blender.run(ret);
	if (CYLINDER)
		return perspective_correction(ret);
	return ret;
}


}	// namepsace stitch
