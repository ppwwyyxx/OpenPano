// File: main.cc
// Date: Wed Jun 17 20:29:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "lib/config.hh"
#include "lib/planedrawer.hh"
#include "lib/imgproc.hh"
#include "warper.hh"
#include "feature/extrema.hh"
#include "feature/keypoint.hh"
#include "lib/timer.hh"
#include "matcher.hh"
#include "panorama.hh"
#include <ctime>
#include <cassert>

using namespace std;

bool TEMPDEBUG = false;

#define LABEL_LEN 7

inline real_t gen_rand()
{ return (real_t)rand() / RAND_MAX; }


void test_feature(const char* fname, int mode = 1) {
	auto mat = read_rgb(fname);
	vector<Descriptor> ans = detect_SIFT(mat);

	PlaneDrawer pld(mat);

	cout << ans.size() << endl;
	for (auto &i : ans) {
		// XXX no dir in feature
		/*
		 *if (mode)
		 *  pld.arrow(toCoor(i.coor), i.dir, LABEL_LEN);
		 *else
		 */
			pld.cross(toCoor(i.coor), LABEL_LEN / 2);
	}
	write_rgb("feature.png", mat);

	// write feature to file
	ofstream feature_out("feature.txt");
	for (auto &i : ans) {
		// in r, c format
		feature_out << int(i.coor.x) << " " << int(i.coor.y) << endl;
	}
	feature_out.close();
}

void test_extrema(const char* fname) {
	auto mat = read_rgb(fname);

	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace dog(ss);
	ExtremaDetector ex(dog);
	//auto extrema = ex.get_raw_extrema();
	auto extrema = ex.get_extrema();

	PlaneDrawer pld(mat);
	for (auto &i : extrema)
		pld.cross(i.real_coor, LABEL_LEN / 2);
	write_rgb("extrema.png", mat);
}


void gallery(const char* f1, const char* f2) {
	list<Mat32f> imagelist;
	Mat32f pic1 = read_rgb(f1);
	Mat32f pic2 = read_rgb(f2);
	imagelist.push_back(pic1);
	imagelist.push_back(pic2);


	vector<Descriptor> feat1 = detect_SIFT(pic1),
										 feat2 = detect_SIFT(pic2);

	Mat32f concatenated = hconcat(imagelist);
	PlaneDrawer pld(concatenated);

	Matcher match(feat1, feat2);
	auto ret = match.match();
	for (auto &x : ret.data) {
		pld.set_color(::Color(gen_rand(), gen_rand(), gen_rand()));
		pld.circle(toCoor(feat1[x.x].coor), LABEL_LEN);
		pld.circle(toCoor(feat2[x.y].coor) + Coor(pic1.width(), 0), LABEL_LEN);
		pld.line(toCoor(feat1[x.x].coor), toCoor(feat2[x.y].coor) + Coor(pic1.width(), 0));
	}
	write_rgb("gallery.png", concatenated);
}

void test_warp(int argc, char* argv[]) {
	Warper warp(1);
	REPL(i, 2, argc) {
		Mat32f mat = read_rgb(argv[i]);
		warp.warp(mat);
		write_rgb((to_string(i) + ".png").c_str(), mat);
	}
}


void work(int argc, char* argv[]) {
	vector<Mat32f> imgs;
	REPL(i, 1, argc)
		imgs.emplace_back(read_rgb(argv[i]));
	Panorama p(imgs);
	Mat32f res = p.get();

	if (CROP) res = crop(res);
	write_rgb("out.png", res);
}

void init_config() {
	ConfigParser Config("config.cfg");
	PANO = Config.get("PANO");
	TRANS = Config.get("TRANS");
	CROP = Config.get("CROP");
	if (PANO && TRANS) {
		cerr << "Want panorama or translation stitching? Cannot have both!" << endl;
		exit(1);
	}
	HOMO = TRANS;

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
	ORI_WINDOW_FACTOR = Config.get("ORI_WINDOW_FACTOR");
	ORI_RADIUS = Config.get("ORI_RADIUS");
	ORT_HIST_SMOOTH_COUNT = Config.get("ORT_HIST_SMOOTH_COUNT");
	ORT_HIST_PEAK_RATIO = Config.get("ORT_HIST_PEAK_RATIO");
	DESC_HIST_REAL_WIDTH = Config.get("DESC_HIST_REAL_WIDTH");
	DESC_NORM_THRESH = Config.get("DESC_NORM_THRESH");
	DESC_INT_FACTOR = Config.get("DESC_INT_FACTOR");
	MATCH_REJECT_NEXT_RATIO = Config.get("MATCH_REJECT_NEXT_RATIO");
	MATCH_MIN_SIZE = Config.get("MATCH_MIN_SIZE");
	CONNECTED_THRES = Config.get("CONNECTED_THRES");
	RANSAC_ITERATIONS = Config.get("RANSAC_ITERATIONS");
	RANSAC_INLIER_THRES = Config.get("RANSAC_INLIER_THRES");
	SLOPE_PLAIN = Config.get("SLOPE_PLAIN");
	OUTPUT_SIZE_FACTOR = Config.get("OUTPUT_SIZE_FACTOR");
}

void planet(const char* fname) {
	Mat32f test = read_rgb(fname);
	int w = test.width(), h = test.height();
	const int OUTSIZE = 1000, center = OUTSIZE / 2;
	Mat32f ret(OUTSIZE, OUTSIZE, 3);
	fill(ret, Color::NO);

	REP(i, OUTSIZE) REP(j, OUTSIZE) {
		real_t dist = hypot(center - i, center - j);
		if (dist >= center || dist == 0) continue;
		dist = dist / center;
		dist = sqr(dist) * dist;
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
		p[0] = c.x, p[1] = c.y, p[2] = c.z;
	}
	write_rgb("planet.png", ret);
}

int main(int argc, char* argv[]) {
	TotalTimerGlobalGuard _g;
	srand(time(NULL));
	init_config();
	string command = argv[1];
	if (command == "extrema")
		test_extrema(argv[2]);
	else if (command == "feature")
		test_feature(argv[2]);
	else if (command == "gallery")
		gallery(argv[2], argv[3]);
	else if (command == "warp")
		test_warp(argc, argv);
	//planet(argv[1]);
	else
		work(argc, argv);
}
