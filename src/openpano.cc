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
#include "openpano.h"
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

void init_config(const char* filename) {
#define CFG(x) \
	x = Config.get(#x)
	const char* config_file = filename;
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

void openpano::stitch(
  const std::string& config_file,
  const std::string& outfile,
  const std::vector<std::string>& infiles,
	bool& success){

  	init_config(config_file.c_str());
  	Mat32f res;
  	if (CYLINDER) {
  		CylinderStitcher p(move(infiles));
  		res = p.build(success);
  	} else {
  		Stitcher p(move(infiles));
  		res = p.build(success);
  	}

  	if (CROP) {
  		int oldw = res.width(), oldh = res.height();
  		res = crop(res);
  		print_debug("Crop from %dx%d to %dx%d\n", oldw, oldh, res.width(), res.height());
  	}
  	{
  		GuardedTimer tm("Writing image");
  		write_rgb(("/data/" + outfile + ".jpg").c_str(), res);
  	}
  }


// void work(int argc, char* argv[]) {
// /*
//  *  vector<Mat32f> imgs(argc - 1);
//  *  {
//  *    GuardedTimer tm("Read images");
//  *#pragma omp parallel for schedule(dynamic)
//  *    REPL(i, 1, argc)
//  *      imgs[i-1] = read_img(argv[i]);
//  *  }
//  */
// 	vector<string> imgs;
// 	REPL(i, 3, argc) imgs.emplace_back(argv[i]);
// 	Mat32f res;
// 	if (CYLINDER) {
// 		CylinderStitcher p(move(imgs));
// 		res = p.build();
// 	} else {
// 		Stitcher p(move(imgs));
// 		res = p.build();
// 	}
//
// 	if (CROP) {
// 		int oldw = res.width(), oldh = res.height();
// 		res = crop(res);
// 		print_debug("Crop from %dx%d to %dx%d\n", oldw, oldh, res.width(), res.height());
// 	}
// 	{
// 		GuardedTimer tm("Writing image");
// 		std::string output_name = string(argv[2]);
// 		write_rgb(("/Superproject/data/" + output_name + ".jpg").c_str(), res);
// 	}
// }
