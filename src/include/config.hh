// File: config.hh
// Date: Sat May 04 22:22:25 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <map>
#include <cstring>
#include <fstream>
#include "common.hh"

class ConfigParser {
	public:
		std::map<std::string, double> data;

		ConfigParser(const char* fname);

		double get(const std::string& s);

};

extern bool TRANS;
extern bool PANO;
extern bool CROP;
extern bool HOMO;

extern int NUM_OCTAVE;
extern int NUM_SCALE;
extern real_t SCALE_FACTOR;

extern real_t GAUSS_SIGMA;
extern int GAUSS_WINDOW_FACTOR;

extern real_t JUDGE_EXTREMA_DIFF_THRES;
extern real_t CONTRAST_THRES;
extern real_t PRE_COLOR_THRES;
extern real_t EDGE_RATIO;

extern int CALC_OFFSET_DEPTH;
extern real_t OFFSET_THRES;

extern real_t ORI_WINDOW_FACTOR;
extern real_t ORI_RADIUS;

extern int ORT_HIST_SMOOTH_COUNT;
extern real_t ORT_HIST_PEAK_RATIO;

extern int DESC_HIST_REAL_WIDTH;
extern real_t DESC_NORM_THRESH;
extern int DESC_INT_FACTOR;

extern real_t MATCH_REJECT_NEXT_RATIO;
extern int MATCH_MIN_SIZE;
extern real_t CONNECTED_THRES;

extern int RANSAC_ITERATIONS;
extern real_t RANSAC_INLIER_THRES;

extern real_t SLOPE_PLAIN;

extern real_t OUTPUT_SIZE_FACTOR;


// keep
const int HOMO_FREEDOM = 8;
const int AFFINE_FREEDOM = 6;

const int ORT_HIST_BIN_NUM = 36;		// lowe

const int DESC_HIST_WIDTH = 4;
const int DESC_HIST_BIN_NUM = 8;
const int DESC_LEN = 128;
