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
float SCALE_FACTOR;

float GAUSS_SIGMA;
int GAUSS_WINDOW_FACTOR;

float JUDGE_EXTREMA_DIFF_THRES;
float CONTRAST_THRES;
float PRE_COLOR_THRES;
float EDGE_RATIO;

int CALC_OFFSET_DEPTH;
float OFFSET_THRES;

float ORI_WINDOW_FACTOR;
float ORI_RADIUS;

int ORT_HIST_SMOOTH_COUNT;
float ORT_HIST_PEAK_RATIO;

int DESC_HIST_SCALE_FACTOR;
float DESC_NORM_THRESH;
int DESC_INT_FACTOR;

float MATCH_REJECT_NEXT_RATIO;
int MATCH_MIN_SIZE;
float CONNECTED_THRES;

int RANSAC_ITERATIONS;
float RANSAC_INLIER_THRES;

float SLOPE_PLAIN;

float OUTPUT_SIZE_FACTOR;
