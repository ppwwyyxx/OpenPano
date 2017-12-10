// File: main.cc
// Date: Wed Jun 17 20:29:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>

#include "openpano.h"
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



const int LABEL_LEN = 7;



int main(int argc, char* argv[]) {


	if (argc <= 4)
		error_exit("Need a config file, output destination, and at least two images to stitch.\n");
	vector<string> imgs;
	REPL(i, 3, argc) imgs.emplace_back(argv[i]);
	openpano::stitch(argv[1], argv[2], imgs);
	// TotalTimerGlobalGuard _g;
	// srand(time(NULL));
	// init_config(argv[1]);
	// string command = argv[3];
	// // if (command == "raw_extrema")
	// // 	test_extrema(argv[2], 0);
	// // else if (command == "keypoint")
	// // 	test_extrema(argv[2], 1);
	// // else if (command == "orientation")
	// // 	test_orientation(argv[2]);
	// // else if (command == "match")
	// // 	test_match(argv[2], argv[3]);
	// // else if (command == "inlier")
	// // 	test_inlier(argv[2], argv[3]);
	// // else if (command == "warp")
	// // 	test_warp(argc, argv);
	// // else if (command == "planet")
	// // 	planet(argv[2]);
	// // else
	// 	// the real routine
	// 	work(argc, argv);
}
