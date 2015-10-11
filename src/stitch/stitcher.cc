// File: stitcher.cc
// Date: Sun Sep 22 12:54:18 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "stitcher.hh"

#include <fstream>
#include <algorithm>
#include <queue>

#include "feature/matcher.hh"
#include "lib/imgproc.hh"
#include "lib/planedrawer.hh"
#include "lib/timer.hh"

#include "blender.hh"
#include "bundle_adjuster.hh"
#include "match_info.hh"
#include "projection.hh"
#include "transform_estimate.hh"
#include "warp.hh"
using namespace std;
using namespace feature;

// in development. estimate camera parameters
bool CAMERA_MODE = true;
const bool DEBUG_OUT = false;

Mat32f Stitcher::build() {
	if (CYLINDER)
		CAMERA_MODE = false;

	calc_feature();
	if (CYLINDER) {
		// TODO choose a better starting point by MST use centrality
		assign_center();
		build_bundle_warp();
		bundle.proj_method = ConnectedImages::ProjectionMethod::flat;
	} else {
		if (CAMERA_MODE)
			pairwise_match();
		else
			assume_linear_pairwise();
		if (DEBUG_OUT)
			debug_matchinfo();
		assign_center();
		if (CAMERA_MODE)
			estimate_camera();
		else
			build_bundle_linear_simple();
		bundle.proj_method = ConnectedImages::ProjectionMethod::cylindrical;
	}
	print_debug("Using projection method: %d\n", bundle.proj_method);
	bundle.update_proj_range();
	return blend();
}

void Stitcher::calc_feature() {
	GuardedTimer tm("calc_feature()");
	int n = imgs.size();
	// detect feature
#pragma omp parallel for schedule(dynamic)
	REP(k, n) {
		feats[k] = feature_det->detect_feature(imgs[k]);
		print_debug("Image %d has %lu features\n", k, feats[k].size());
	}
}

void Stitcher::pairwise_match() {
	GuardedTimer tm("pairwise_match()");
	size_t n = imgs.size();

	REP(i, n) REPL(j, i + 1, n) {
		FeatureMatcher matcher(feats[i], feats[j]);
		auto match = matcher.match();
		TransformEstimation transf(match, feats[i], feats[j]);
		MatchInfo info;
		bool succ = transf.get_transform(&info);
		if (not succ) {
			//print_debug("Only %f match from %lu to %lu\n", info.confidence, i, j);
			continue;
		}
		auto inv = info.homo.inverse(&succ);
		if (not succ) continue;	// cannot inverse. mal-formed homography
		print_debug(
				"Connection between image %lu and %lu, ninliers=%lu, conf=%f\n",
				i, j, info.match.size(), info.confidence);
		//cout << "Estimated H" << info.homo << endl;
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
	REP(i, n-1) {
		int next = (i + 1) % n;
		FeatureMatcher matcher(feats[i], feats[next]);
		auto match = matcher.match();
		TransformEstimation transf(match, feats[i], feats[next]);
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
}

void Stitcher::estimate_camera() {
	int n = imgs.size();
	{ // assign an initial focal length
		double focal = Camera::estimate_focal(pairwise_matches);
		if (focal > 0) {
			for (auto& c : cameras)
				c.focal = focal;
			print_debug("Estimated focal: %lf\n", focal);
		} else
			REP(i, n) // hack focal
				cameras[i].focal = (imgs[i].width() / imgs[i].height()) * 0.5;
	}
	vector<vector<int>> graph;
	max_spanning_tree(graph);

	int start = bundle.identity_idx;
	queue<int> q; q.push(start);
	vector<bool> vst(graph.size(), false);		// in queue
	vst[start] = true;
	while (q.size()) {
		int now = q.front(); q.pop();
		for (int next: graph[now]) {
			if (vst[next]) continue;
			vst[next] = true;
			// from now to next
			auto Kfrom = cameras[now].K();
			auto Kto = cameras[next].K();
			auto Hinv = pairwise_matches[now][next].homo;
			auto Mat = Kfrom.inverse() * Hinv * Kto;
			cameras[next].R = cameras[now].R * Mat;
			// XXX this R is actually R.inv. and also in the final construction in H
			// but it goes like this in opencv
			// this is the R going from this image to identity
			q.push(next);
		}
	}
	REP(i, n) {
		cameras[i].ppx = imgs[i].width() / 2;
		cameras[i].ppy = imgs[i].height() / 2;
	}

	BundleAdjuster ba(imgs, pairwise_matches);
 	ba.estimate(cameras);
	if (STRAIGHTEN)
		Camera::straighten(cameras);
	// TODO rotate to identity
	REP(i, n) {
		bundle.component[i].homo_inv = cameras[i].K() * cameras[i].R.transpose();
		bundle.component[i].homo = cameras[i].R * cameras[i].K().inverse();
	}
}


void Stitcher::build_bundle_linear_simple() {
	// assume pano pairwise
	int n = imgs.size(), mid = bundle.identity_idx;
	bundle.component[mid].homo = Homography::I();

	auto& comp = bundle.component;

	// accumulate the transformations
	comp[mid+1].homo = pairwise_matches[mid][mid+1].homo;
	REPL(k, mid + 2, n)
		comp[k].homo = Homography(
				comp[k - 1].homo.prod(pairwise_matches[k-1][k].homo));
	comp[mid-1].homo = pairwise_matches[mid][mid-1].homo;
	REPD(k, mid - 2, 0)
		comp[k].homo = Homography(
				comp[k + 1].homo.prod(pairwise_matches[k+1][k].homo));
	// then, comp[k]: from k to identity
	bundle.calc_inverse_homo();
}


void Stitcher::build_bundle_warp() {;
	GuardedTimer tm("build_bundle_warp()");
	int n = imgs.size(), mid = bundle.identity_idx;
	REP(i, n) bundle.component[i].homo = Homography::I();

	Timer timer;
	vector<MatchData> matches;		// matches[k]: k,k+1
	matches.resize(n-1);
	REP(k, n - 1) {
		FeatureMatcher matcher(feats[k], feats[(k + 1) % n]);
		matches[k] = matcher.match();
	}
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
				matches[i], feats[i + 1], feats[i]).get_transform(&info);
		if (not succ)
			error_exit(ssprintf("Image %d and %d doesn't match. Failed", i, i+1));
		bundle.component[i].homo = info.homo;
	}
	REPD(i, mid - 2, 0)
		bundle.component[i].homo = Homography(
				bundle.component[i + 1].homo.prod(bundle.component[i].homo));
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
				nowfeats[k]).get_transform(&info);
		if (not succ)
			failed = true;
			//error_exit("The two image doesn't match. Failed");
		nowmat[k-1] = info.homo;
	}
	if (failed) return 0;

	REPL(k, 1, len - 1)
		nowmat[k] = nowmat[k - 1].prod(nowmat[k]);	// transform to nowimgs[0] == imgs[mid]

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
	Mat<Vec2D> orig_pos(h, w, 1);
	REP(i, h) REP(j, w) {
		Vec2D& p = (orig_pos.at(i, j) = inv.trans2d(Vec2D(j, i)));
		if (!p.isNaN() && (p.x < 0 || p.x >= w || p.y < 0 || p.y >= h))
			p = Vec2D::NaN();
	}
	blender.add_image(Coor(0, 0), orig_pos, img);
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
				 x_per_pixel = id_img_range.x / (CYLINDER ? refw : refh),	// huh?
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
	for (auto& cur : bundle.component) {
		Coor top_left = scale_coor_to_img_coor(cur.range.min);
		Coor bottom_right = scale_coor_to_img_coor(cur.range.max);
		Coor diff = bottom_right - top_left;
		int w = diff.x, h = diff.y;
		Mat<Vec2D> orig_pos(h, w, 1);

		REP(i, h) REP(j, w) {
			Vec2D c((j + top_left.x) * x_per_pixel + proj_min.x, (i + top_left.y) * y_per_pixel + proj_min.y);
			Vec homo = proj2homo(Vec2D(c.x / refw, c.y / refh));
			if (not CAMERA_MODE)  {	// scale and offset is in camera intrinsic
				homo.x -= 0.5 * homo.z, homo.y -= 0.5 * homo.z;	// shift center for homography
				homo.x *= refw, homo.y *= refh;
			}
			Vec2D orig = cur.homo_inv.trans_normalize(homo);
			if (not CAMERA_MODE)
				orig = orig + Vec2D(cur.imgptr->width()/2, cur.imgptr->height()/2);
			Vec2D& p = (orig_pos.at(i, j) = orig);
			if (!p.isNaN() && (p.x < 0 || p.x >= cur.imgptr->width()
						|| p.y < 0 || p.y >= cur.imgptr->height()))
				p = Vec2D::NaN();
		}
		blender.add_image(top_left, orig_pos, *cur.imgptr);
	}
	if (DEBUG_OUT)
		blender.debug_run(size.x, size.y);
	blender.run(ret);
	if (CYLINDER)
		return perspective_correction(ret);
	return ret;
}


void Stitcher::debug_matchinfo() {
	int n = imgs.size();
	REP(i, n) REPL(j, i+1, n) {
		auto& m = pairwise_matches[j][i];
		if (m.confidence <= 0) continue;
		print_debug("Dump matchinfo of %d->%d\n", i, j);
		list<Mat32f> imagelist{imgs[i], imgs[j]};
		Mat32f conc = vconcat(imagelist);
		PlaneDrawer pld(conc);
		for (auto& p : m.match) {
			pld.set_rand_color();
			Coor icoor1 = Coor(p.second.x + imgs[i].width()/2,
					p.second.y + imgs[i].height()/2);
			Coor icoor2 = Coor(p.first.x + imgs[j].width()/2,
					p.first.y + imgs[j].height()/2);
			pld.circle(icoor1, 7);
			pld.circle(icoor2 + Coor(0, imgs[i].height()), 7);
			pld.line(icoor1, icoor2 + Coor(0, imgs[i].height()));
		}
		write_rgb(ssprintf("/t/match%d-%d.png", i, j).c_str(), conc);
	}
}

bool Stitcher::max_spanning_tree(vector<vector<int>>& graph) {
	struct Edge {
		int v1, v2;
		float weight;
		bool have(int v) { return v1 == v || v2 == v; }
		Edge(int a, int b, float v):v1(a), v2(b), weight(v) {}
		bool operator < (const Edge& r) const
		{ return weight > r.weight;	}
	};

	int n = imgs.size();
	graph.clear(); graph.resize(n);
	vector<Edge> edges;
	REP(i, n) REPL(j, i+1, n) {
		auto& m = pairwise_matches[i][j];
		if (m.confidence <= 0) continue;
		edges.emplace_back(i, j, m.confidence);
	}
	sort(edges.begin(), edges.end());		// large weight to small weight
	vector<bool> in_tree(n, false);
	int edge_cnt = 0;
	in_tree[edges.front().v1] = true;
	while (true) {
		int old_edge_cnt = edge_cnt;
		auto itr = begin(edges);
		for (; itr != edges.end(); ++itr) {
			Edge& e = *itr;
			if (in_tree[e.v1] && in_tree[e.v2]) {
				edges.erase(itr);
				break;
			}
			if (not in_tree[e.v1] && not in_tree[e.v2])
				continue;
			in_tree[e.v1] = in_tree[e.v2] = true;
			graph[e.v1].push_back(e.v2);
			graph[e.v2].push_back(e.v1);
			print_debug("MST: Best edge from %d to %d\n", e.v1, e.v2);
			edges.erase(itr);
			edge_cnt ++;
			break;
		}
		if (edge_cnt == n - 1) // tree is full
			break;
		if (edge_cnt == old_edge_cnt && itr == edges.end())
			// no edge to add
			break;
	}
	if (edge_cnt != n - 1) {
		print_debug("Found a tree of size %d!=%d, images not connected!",
				edge_cnt, n - 1);
		abort();
	}
	return true;
}
