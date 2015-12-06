// File: config.hh
// Date: Sat May 04 22:22:25 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <map>
#include <cstring>
#include <fstream>
#include "common.hh"

namespace config {

class ConfigParser {
	public:
		std::map<std::string, double> data;

		ConfigParser(const char* fname);

		double get(const std::string& s);

};

extern bool CYLINDER;
extern bool TRANS;
extern bool CROP;
extern float FOCAL_LENGTH;
extern bool ESTIMATE_CAMERA;
extern bool STRAIGHTEN;
extern bool MULTIPASS_BA;

extern int SIFT_WORKING_SIZE;
extern int NUM_OCTAVE;
extern int NUM_SCALE;
extern float SCALE_FACTOR;

extern float GAUSS_SIGMA;
extern int GAUSS_WINDOW_FACTOR;

extern float JUDGE_EXTREMA_DIFF_THRES;
extern float CONTRAST_THRES;
extern float PRE_COLOR_THRES;
extern float EDGE_RATIO;

extern int CALC_OFFSET_DEPTH;
extern float OFFSET_THRES;

extern float ORI_RADIUS;
extern int ORI_HIST_SMOOTH_COUNT;

extern int DESC_HIST_SCALE_FACTOR;
extern int DESC_INT_FACTOR;

extern float MATCH_REJECT_NEXT_RATIO;
extern int MATCH_MIN_SIZE;
extern float CONNECTED_THRES;

extern int RANSAC_ITERATIONS;
extern double RANSAC_INLIER_THRES;

extern float SLOPE_PLAIN;



// keep unchanged
const float ORI_WINDOW_FACTOR = 1.5f;
const int ORI_HIST_BIN_NUM = 36;		// lowe
const float ORI_HIST_PEAK_RATIO = 0.8f;

const int DESC_HIST_WIDTH = 4;
const int DESC_HIST_BIN_NUM = 8;
const int DESC_LEN = 128;	// (4x4)x8
const float DESC_NORM_THRESH = 0.2f;

const int BRIEF_PATH_SIZE = 9;
const int BRIEF_NR_PAIR = 256;

const int FLANN_NR_KDTREE = 4;

}
