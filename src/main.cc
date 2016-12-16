// File: main.cc
// Date: Wed Jun 17 20:29:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>

#include "feature/extrema.hh"
#include "feature/matcher.hh"
#include "feature/orientation.hh"
#include "lib/mat.h"
#include "lib/config.hh"
#include "lib/geometry.hh"
#include "lib/imgproc.hh"
#include "lib/planedrawer.hh"
#include "lib/polygon.hh"
#include "lib/timer.hh"
#include "stitch/cylstitcher.hh"
#include "stitch/match_info.hh"
#include "stitch/stitcher.hh"
#include "stitch/transform_estimate.hh"
#include "stitch/warp.hh"
#include "common/common.hh"
#include <ctime>
#include <cassert>

#ifdef DISABLE_JPEG
#define IMGFILE(x) #x ".png"
#else
#define IMGFILE(x) #x ".jpg"
#endif

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
	write_rgb(IMGFILE(extrema), mat);
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
	write_rgb(IMGFILE(orientation), mat);
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

	Mat32f concatenated = hconcat(imagelist);
	PlaneDrawer pld(concatenated);

	FeatureMatcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("Match size: %d\n", ret.size());
	for (auto &x : ret.data) {
		pld.set_rand_color();
		Vec2D coor1 = feat1[x.first].coor,
					coor2 = feat2[x.second].coor;
		Coor icoor1 = Coor(coor1.x + pic1.width()/2, coor1.y + pic1.height()/2);
		Coor icoor2 = Coor(coor2.x + pic2.width()/2 + pic1.width(), coor2.y + pic2.height()/2);
		pld.circle(icoor1, LABEL_LEN);
		pld.circle(icoor2, LABEL_LEN);
		pld.line(icoor1, icoor2);
	}
	write_rgb(IMGFILE(match), concatenated);
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
	vector<Vec2D> kp1; for (auto& d : feat1) kp1.emplace_back(d.coor);
	vector<Vec2D> kp2; for (auto& d : feat2) kp2.emplace_back(d.coor);
	print_debug("Feature: %lu, %lu\n", feat1.size(), feat2.size());

	Mat32f concatenated = hconcat(imagelist);
	PlaneDrawer pld(concatenated);
	FeatureMatcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("Match size: %d\n", ret.size());

	TransformEstimation est(ret, kp1, kp2,
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

	write_rgb(IMGFILE(inlier), concatenated);
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
/*
 *  vector<Mat32f> imgs(argc - 1);
 *  {
 *    GuardedTimer tm("Read images");
 *#pragma omp parallel for schedule(dynamic)
 *    REPL(i, 1, argc)
 *      imgs[i-1] = read_img(argv[i]);
 *  }
 */
	vector<string> imgs;
	REPL(i, 1, argc) imgs.emplace_back(argv[i]);
	Mat32f res;
	if (CYLINDER) {
		CylinderStitcher p(move(imgs));
		res = p.build();
	} else {
		Stitcher p(move(imgs));
		res = p.build();
	}

	if (CROP) {
		int oldw = res.width(), oldh = res.height();
		res = crop(res);
		print_debug("Crop from %dx%d to %dx%d\n", oldw, oldh, res.width(), res.height());
	}
	{
		GuardedTimer tm("Writing image");
		write_rgb(IMGFILE(out), res);
	}
}

void init_config() {
#define CFG(x) \
	x = Config.get(#x)
	const char* config_file = "config.cfg";
	ConfigParser Config(config_file);
	CFG(CYLINDER);
	CFG(TRANS);
	CFG(ESTIMATE_CAMERA);
	if (int(CYLINDER) + int(TRANS) + int(ESTIMATE_CAMERA) >= 2)
		error_exit("You set two many modes...\n");
	if (CYLINDER)
		print_debug("Run with cylinder mode.\n");
	else if (TRANS)
		print_debug("Run with translation mode.\n");
	else if (ESTIMATE_CAMERA)
		print_debug("Run with camera estimation mode.\n");
	else
		print_debug("Run with naive mode.\n");

	CFG(ORDERED_INPUT);
	if (!ORDERED_INPUT && !ESTIMATE_CAMERA)
		error_exit("Require ORDERED_INPUT under this mode!\n");

	CFG(CROP);
	CFG(STRAIGHTEN);
	CFG(FOCAL_LENGTH);
	CFG(MAX_OUTPUT_SIZE);
	CFG(LAZY_READ);	// TODO in cyl mode

	CFG(SIFT_WORKING_SIZE);
	CFG(NUM_OCTAVE);
	CFG(NUM_SCALE);
	CFG(SCALE_FACTOR);
	CFG(GAUSS_SIGMA);
	CFG(GAUSS_WINDOW_FACTOR);
	CFG(JUDGE_EXTREMA_DIFF_THRES);
	CFG(CONTRAST_THRES);
	CFG(PRE_COLOR_THRES);
	CFG(EDGE_RATIO);
	CFG(CALC_OFFSET_DEPTH);
	CFG(OFFSET_THRES);
	CFG(ORI_RADIUS);
	CFG(ORI_HIST_SMOOTH_COUNT);
	CFG(DESC_HIST_SCALE_FACTOR);
	CFG(DESC_INT_FACTOR);
	CFG(MATCH_REJECT_NEXT_RATIO);
	CFG(RANSAC_ITERATIONS);
	CFG(RANSAC_INLIER_THRES);
	CFG(INLIER_IN_MATCH_RATIO);
	CFG(INLIER_IN_POINTS_RATIO);
	CFG(SLOPE_PLAIN);
	CFG(LM_LAMBDA);
	CFG(MULTIPASS_BA);
	CFG(MULTIBAND);
#undef CFG
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
	write_rgb(IMGFILE(planet), ret);
}

int main(int argc, char* argv[]) {
	if (argc <= 2)
		error_exit("Need at least two images to stitch.\n");
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
