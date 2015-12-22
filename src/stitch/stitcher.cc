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
#include "camera.hh"
#include "warp.hh"
using namespace std;
using namespace pano;
using namespace config;

namespace pano {

// use in development
const static bool DEBUG_OUT = false;
const static char* MATCHINFO_DUMP = "log/matchinfo.txt";

Mat32f Stitcher::build() {
	calc_feature();
	// TODO choose a better starting point by MST use centrality
	if (TRANS)
		assume_linear_pairwise();
	else
		pairwise_match();
	free_feature();
	//load_matchinfo(MATCHINFO_DUMP);
	if (DEBUG_OUT) {
		draw_matchinfo();
		dump_matchinfo(MATCHINFO_DUMP);
	}
	assign_center();

	if (ESTIMATE_CAMERA)
		estimate_camera();
	else
		build_linear_simple();		// naive mode
	// TODO automatically determine projection method
	if (ESTIMATE_CAMERA)
		bundle.proj_method = ConnectedImages::ProjectionMethod::cylindrical;
	else
		bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
	print_debug("Using projection method: %d\n", bundle.proj_method);
	bundle.update_proj_range();
	return bundle.blend();
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
	REP(k, (int)tasks.size()) {
		int i = tasks[k].first, j = tasks[k].second;
		//auto match = FeatureMatcher(feats[i], feats[j]).match();	// slow
		auto match = pwmatcher.match(i, j);
		TransformEstimation transf(match, keypoints[i], keypoints[j],
				{imgs[i].width(), imgs[i].height()},
				{imgs[j].width(), imgs[j].height()});	// from j to i
		MatchInfo info;
		bool succ = transf.get_transform(&info);
		if (!succ) {
			if (-(int)info.confidence >= 8)	// reject for geometry reason
				print_debug("Reject bad match with %d inlier from %d to %d\n", -(int)info.confidence, i, j);
			continue;
		}
		auto inv = info.homo.inverse();	// TransformEstimation ensures invertible
		inv.mult(1.0 / inv[8]);	// TODO more stable?
		print_debug(
				"Connection between image %d and %d, ninliers=%lu/%d=%lf, conf=%f\n",
				i, j, info.match.size(), match.size(), info.match.size() * 1.0 / match.size(), info.confidence);
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
		TransformEstimation transf(match, keypoints[i], keypoints[next],
				{imgs[i].width(), imgs[i].height()},
				{imgs[next].width(), imgs[next].height()});
		MatchInfo info;
		bool succ = transf.get_transform(&info);
		if (! succ)
			error_exit(ssprintf("Image %d and %d doesn't match.\n", i, next));
		auto inv = info.homo.inverse(&succ);
		if (! succ) // cannot inverse. mal-formed homography
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
	auto cameras = CameraEstimator{pairwise_matches, shapes}.estimate();

	// produced homo operates on [0,w] coordinate
	REP(i, imgs.size()) {
		bundle.component[i].homo_inv = cameras[i].K() * cameras[i].R;
		bundle.component[i].homo = cameras[i].Rinv() * cameras[i].K().inverse();
	}
}

void Stitcher::build_linear_simple() {
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
	// now, comp[k]: from k to identity

	bundle.shift_all_homo();
	bundle.calc_inverse_homo();
}

}	// namepsace stitch
