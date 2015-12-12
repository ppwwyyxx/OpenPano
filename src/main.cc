// File: main.cc
// Date: Wed Jun 17 20:29:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>

#include "feature/extrema.hh"
#include "feature/matcher.hh"
#include "feature/orientation.hh"
#include "lib/config.hh"
#include "lib/geometry.hh"
#include "lib/imgproc.hh"
#include "lib/planedrawer.hh"
#include "lib/polygon.hh"
#include "lib/timer.hh"
#include "stitch/match_info.hh"
#include "stitch/stitcher.hh"
#include "stitch/transform_estimate.hh"
#include "stitch/warp.hh"
#include <ctime>
#include <cassert>

using namespace std;
using namespace pano;
using namespace config;

bool TEMPDEBUG = false;

const int LABEL_LEN = 7;

void test_extrema(const char* fname, int mode) {
	auto mat = read_img(fname);

	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace dog(ss);
	ExtremaDetector ex(dog);

	PlaneDrawer pld(mat);
	if (mode == 0) {
		auto extrema = ex.get_raw_extrema();
		PP(extrema.size());
		for (auto &i : extrema)
			pld.cross(i, LABEL_LEN / 2);
	} else if (mode == 1) {
		auto extrema = ex.get_extrema();
		cout << extrema.size() << endl;
		for (auto &i : extrema) {
			Coor c{(int)(i.real_coor.x * mat.width()), (int)(i.real_coor.y * mat.height())};
			pld.cross(c, LABEL_LEN / 2);
		}
	}
	write_rgb("extrema.jpg", mat);
}

void test_orientation(const char* fname) {
	auto mat = read_img(fname);
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace dog(ss);
	ExtremaDetector ex(dog);
	auto extrema = ex.get_extrema();
	OrientationAssign ort(dog, ss, extrema);
	auto oriented_keypoint = ort.work();

	PlaneDrawer pld(mat);
	pld.set_rand_color();

	cout << "FeaturePoint size: " << oriented_keypoint.size() << endl;
	for (auto &i : oriented_keypoint)
		pld.arrow(Coor(i.real_coor.x * mat.width(), i.real_coor.y * mat.height()), i.dir, LABEL_LEN);
	write_rgb("orientation.jpg", mat);
}

// draw feature and their match
void test_match(const char* f1, const char* f2) {
	list<Mat32f> imagelist;
	Mat32f pic1 = read_img(f1);
	Mat32f pic2 = read_img(f2);
	imagelist.push_back(pic1);
	imagelist.push_back(pic2);

	unique_ptr<FeatureDetector> detector;
	detector.reset(new SIFTDetector);
	vector<Descriptor> feat1 = detector->detect_feature(pic1),
										 feat2 = detector->detect_feature(pic2);
	print_debug("Feature: %lu, %lu\n", feat1.size(), feat2.size());

	Mat32f concatenated = vconcat(imagelist);
	PlaneDrawer pld(concatenated);

	FeatureMatcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("Match size: %d\n", ret.size());
	for (auto &x : ret.data) {
		pld.set_rand_color();
		Vec2D coor1 = feat1[x.first].coor,
					coor2 = feat2[x.second].coor;
		Coor icoor1 = Coor(coor1.x + pic1.width()/2, coor1.y + pic1.height()/2);
		Coor icoor2 = Coor(coor2.x + pic2.width()/2, coor2.y + pic2.height()/2);
		pld.circle(icoor1, LABEL_LEN);
		pld.circle(icoor2 + Coor(0, pic1.height()), LABEL_LEN);
		pld.line(icoor1, icoor2 + Coor(0, pic1.height()));
	}
	write_rgb("match.jpg", concatenated);
}

// draw inliers of the estimated homography
void test_inlier(const char* f1, const char* f2) {
	list<Mat32f> imagelist;
	Mat32f pic1 = read_img(f1);
	Mat32f pic2 = read_img(f2);
	imagelist.push_back(pic1);
	imagelist.push_back(pic2);

	unique_ptr<FeatureDetector> detector;
	detector.reset(new SIFTDetector);
	vector<Descriptor> feat1 = detector->detect_feature(pic1),
										 feat2 = detector->detect_feature(pic2);
	print_debug("Feature: %lu, %lu\n", feat1.size(), feat2.size());

	Mat32f concatenated = hconcat(imagelist);
	PlaneDrawer pld(concatenated);
	FeatureMatcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("Match size: %d\n", ret.size());

	TransformEstimation est(ret, feat1, feat2,
			{pic1.width(), pic1.height()}, {pic2.width(), pic2.height()});
	MatchInfo info;
	est.get_transform(&info);
	print_debug("Inlier size: %lu, conf=%lf\n", info.match.size(), info.confidence);
	if (info.match.size() == 0)
		return;

	for (auto &x : info.match) {
		pld.set_rand_color();
		Vec2D coor1 = x.first,
					coor2 = x.second;
		Coor icoor1 = Coor(coor1.x + pic1.width()/2, coor1.y + pic1.height()/2);
		Coor icoor2 = Coor(coor2.x + pic2.width()/2, coor2.y + pic2.height()/2);
		pld.circle(icoor1, LABEL_LEN);
		pld.circle(icoor2 + Coor(pic1.width(), 0), LABEL_LEN);
		pld.line(icoor1, icoor2 + Coor(pic1.width(), 0));
	}
	pld.set_color(Color(0,0,0));
	Vec2D offset1(pic1.width()/2, pic1.height()/2);
	Vec2D offset2(pic2.width()/2 + pic1.width(), pic2.height()/2);

	// draw convex hull of inliers
	/*
	 *vector<Vec2D> pts1, pts2;
	 *for (auto& x : info.match) {
	 *  pts1.emplace_back(x.first + offset1);
	 *  pts2.emplace_back(x.second + offset2, 0));
	 *}
	 *auto hull = convex_hull(pts1);
	 *pld.polygon(hull);
	 *hull = convex_hull(pts2);
	 *pld.polygon(hull);
	 */

	// draw warped four edges
	Shape2D shape2{pic2.width(), pic2.height()}, shape1{pic1.width(), pic1.height()};
	/*
	 *vector<Vec2D> poly2in1, poly1in2;
	 *auto transform = info.homo;
	 *PP(transform);
	 *poly2in1.emplace_back(offset1 + transform.trans2d(Vec2D{-shape2.halfw(), -shape2.halfh()}));
	 *poly2in1.emplace_back(offset1 + transform.trans2d(Vec2D{-shape2.halfw(), shape2.halfh()}));
	 *poly2in1.emplace_back(offset1 + transform.trans2d(shape2.center()));
	 *poly2in1.emplace_back(offset1 + transform.trans2d(Vec2D{shape2.halfw(), -shape2.halfh()}));
	 *PA(poly2in1);
	 *REP(i, shape2.w) {
	 *  Vec2D pin2{(double)i - shape2.halfw(), (double)i - shape2.halfh()};
	 *  Vec2D pin1 = transform.trans2d(pin2) + offset1;
	 *  pld.cross(pin1, 10);
	 *  pin2 = Vec2D{(double)i - shape2.halfw(), shape2.h- shape2.w + (double)i - shape2.halfh()};
	 *  pin1 = transform.trans2d(pin2) + offset1;
	 *  pld.cross(pin1, 10);
	 *}
	 *auto inv = transform.inverse();
	 *PP(inv);
	 *poly1in2.emplace_back(offset2 + inv.trans2d(Vec2D{-shape1.halfw(), -shape1.halfh()}));
	 *poly1in2.emplace_back(offset2 + inv.trans2d(Vec2D{-shape1.halfw(), shape1.halfh()}));
	 *poly1in2.emplace_back(offset2 + inv.trans2d(shape1.center()));
	 *poly1in2.emplace_back(offset2 + inv.trans2d(Vec2D{shape1.halfw(), -shape1.halfh()}));
	 *PA(poly1in2);
	 *pld.polygon(poly2in1);
	 *pld.polygon(poly1in2);
	 */

	// draw overlapping region
	Matrix homo(3,3);
	REP(i, 9) homo.ptr()[i] = info.homo[i];
	Homography inv = info.homo.inverse();
	auto p = overlap_region(shape1, shape2, homo, inv);
	PA(p);
	for (auto& v: p) v += offset1;
	pld.polygon(p);

	Matrix invM(3, 3);
	REP(i, 9) invM.ptr()[i] = inv[i];
	p = overlap_region(shape2, shape1, invM, info.homo);
	PA(p);
	for (auto& v: p) v += offset2;
	pld.polygon(p);

	write_rgb("inlier.jpg", concatenated);
}

void test_warp(int argc, char* argv[]) {
	CylinderWarper warp(1);
	REPL(i, 2, argc) {
		Mat32f mat = read_img(argv[i]);
		warp.warp(mat);
		write_rgb(("warp" + to_string(i) + ".jpg").c_str(), mat);
	}
}


void work(int argc, char* argv[]) {
	vector<Mat32f> imgs;
	REPL(i, 1, argc)
		imgs.emplace_back(read_img(argv[i]));
	Stitcher p(move(imgs));
	Mat32f res = p.build();
	if (res.width() * res.height() > 600000000 || max(res.width(), res.height()) > 30000)
		error_exit("Result too large. Something must be wrong\n");
	if (res.width() * res.height() > 12000000) {
		print_debug("Result is large, resizing for faster output...\n");
		float ratio = max(res.width(), res.height()) * 1.0f / 8000;
		Mat32f dst(res.height() * 1.0f / ratio, res.width() * 1.0f / ratio, 3);
		resize(res, dst);
		res = dst;
	}

	if (CROP) {
		int oldw = res.width(), oldh = res.height();
		res = crop(res);
		print_debug("Crop from %dx%d to %dx%d\n", oldh, oldw, res.height(), res.width());
	}
	{
		GuardedTimer tm("Writing image");
		write_rgb("out.jpg", res);
	}
}

void init_config() {
	const char* config_file = "config.cfg";
	ConfigParser Config(config_file);
	CYLINDER = Config.get("CYLINDER");
	TRANS = Config.get("TRANS");
	ESTIMATE_CAMERA = Config.get("ESTIMATE_CAMERA");
	if (int(CYLINDER) + int(TRANS) + int(ESTIMATE_CAMERA) >= 2)
		error_exit("You set two many modes...");
	if (CYLINDER)
		print_debug("Run with cylinder mode.\n");
	else if (TRANS)
		print_debug("Run with translation mode.\n");
	else if (ESTIMATE_CAMERA)
		print_debug("Run with camera estimation mode.\n");
	else
		print_debug("Run with naive mode.\n");

	CROP = Config.get("CROP");
	STRAIGHTEN = Config.get("STRAIGHTEN");
	FOCAL_LENGTH = Config.get("FOCAL_LENGTH");
	MULTIPASS_BA = Config.get("MULTIPASS_BA");
	LM_LAMBDA = Config.get("LM_LAMBDA");

	SIFT_WORKING_SIZE = Config.get("SIFT_WORKING_SIZE");
	NUM_OCTAVE = Config.get("NUM_OCTAVE");
	NUM_SCALE = Config.get("NUM_SCALE");
	SCALE_FACTOR = Config.get("SCALE_FACTOR");
	GAUSS_SIGMA = Config.get("GAUSS_SIGMA");
	GAUSS_WINDOW_FACTOR = Config.get("GAUSS_WINDOW_FACTOR");
	JUDGE_EXTREMA_DIFF_THRES = Config.get("JUDGE_EXTREMA_DIFF_THRES");
	CONTRAST_THRES = Config.get("CONTRAST_THRES");
	PRE_COLOR_THRES = Config.get("PRE_COLOR_THRES");
	EDGE_RATIO = Config.get("EDGE_RATIO");
	CALC_OFFSET_DEPTH = Config.get("CALC_OFFSET_DEPTH");
	OFFSET_THRES = Config.get("OFFSET_THRES");
	ORI_RADIUS = Config.get("ORI_RADIUS");
	ORI_HIST_SMOOTH_COUNT = Config.get("ORI_HIST_SMOOTH_COUNT");
	DESC_HIST_SCALE_FACTOR = Config.get("DESC_HIST_SCALE_FACTOR");
	DESC_INT_FACTOR = Config.get("DESC_INT_FACTOR");
	MATCH_REJECT_NEXT_RATIO = Config.get("MATCH_REJECT_NEXT_RATIO");
	RANSAC_ITERATIONS = Config.get("RANSAC_ITERATIONS");
	RANSAC_INLIER_THRES = Config.get("RANSAC_INLIER_THRES");
	INLIER_MINIMUM_RATIO = Config.get("INLIER_MINIMUM_RATIO");
	SLOPE_PLAIN = Config.get("SLOPE_PLAIN");
}

void planet(const char* fname) {
	Mat32f test = read_img(fname);
	int w = test.width(), h = test.height();
	const int OUTSIZE = 1000, center = OUTSIZE / 2;
	Mat32f ret(OUTSIZE, OUTSIZE, 3);
	fill(ret, Color::NO);

	REP(i, OUTSIZE) REP(j, OUTSIZE) {
		real_t dist = hypot(center - i, center - j);
		if (dist >= center || dist == 0) continue;
		dist = dist / center;
		//dist = sqr(dist);	// TODO you can change this to see different effect
		dist = h - dist * h;

		real_t theta;
		if (j == center) {
			if (i < center)
				theta = M_PI / 2;
			else
				theta = 3 * M_PI / 2;
		} else {
			theta = atan((real_t)(center - i) / (center - j));
			if (theta < 0) theta += M_PI;
			if ((theta == 0) && (j > center)) theta += M_PI;
			if (center < i) theta += M_PI;
		}
		m_assert(0 <= theta);
		m_assert(2 * M_PI + EPS >= theta);

		theta = theta / (M_PI * 2) * w;

		update_min(dist, (real_t)h - 1);
		Color c = interpolate(test, dist, theta);
		float* p = ret.ptr(i, j);
		c.write_to(p);
	}
	write_rgb("planet.jpg", ret);
}

int main(int argc, char* argv[]) {
	TotalTimerGlobalGuard _g;
	srand(time(NULL));
	init_config();

	string command = argv[1];
	if (command == "raw_extrema")
		test_extrema(argv[2], 0);
	else if (command == "keypoint")
		test_extrema(argv[2], 1);
	else if (command == "orientation")
		test_orientation(argv[2]);
	else if (command == "match")
		test_match(argv[2], argv[3]);
	else if (command == "inlier")
		test_inlier(argv[2], argv[3]);
	else if (command == "warp")
		test_warp(argc, argv);
	else if (command == "planet")
		planet(argv[2]);
	else
		// the real routine
		work(argc, argv);
}
