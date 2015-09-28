// File: config.cc
// Date: Sat May 04 22:22:20 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "config.hh"
using namespace std;

ConfigParser::ConfigParser(const char* fname) {
	ifstream fin(fname);
	string s;
	double val;
	while (fin >> s) {
		if (s[0] == '#') {
			fin.getline(&s[0], 255, '\n');
			continue;
		}
		fin >> val;
		data[s] = val;
	}
}

double ConfigParser::get(const std::string& s) {
	return data[s];
}

bool TRANS;
bool PANO;
bool HOMO;
bool CROP = true;

int NUM_OCTAVE;
int NUM_SCALE;
real_t SCALE_FACTOR;

real_t GAUSS_SIGMA;
int GAUSS_WINDOW_FACTOR;

real_t JUDGE_EXTREMA_DIFF_THRES;
real_t CONTRAST_THRES;
real_t PRE_COLOR_THRES;
real_t EDGE_RATIO;

int CALC_OFFSET_DEPTH;
real_t OFFSET_THRES;

real_t ORI_WINDOW_FACTOR;
real_t ORI_RADIUS;

int ORT_HIST_SMOOTH_COUNT;
real_t ORT_HIST_PEAK_RATIO;

int DESC_HIST_REAL_WIDTH;
real_t DESC_NORM_THRESH;
int DESC_INT_FACTOR;

real_t MATCH_REJECT_NEXT_RATIO;
int MATCH_MIN_SIZE;
real_t CONNECTED_THRES;

int RANSAC_ITERATIONS;
real_t RANSAC_INLIER_THRES;

real_t SLOPE_PLAIN;

real_t OUTPUT_SIZE_FACTOR;
