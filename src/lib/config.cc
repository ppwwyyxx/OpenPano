// File: config.cc
// Date: Sat May 04 22:22:20 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "config.hh"
#include "debugutils.hh"
#include "utils.hh"
using namespace std;

namespace config {

// TODO allow different types for a value. using template
ConfigParser::ConfigParser(const char* fname) {
	if (! exists_file(fname))
		error_exit("Cannot find config file!");
	const static size_t BUFSIZE = 4096;		// TODO overflow
	ifstream fin(fname);
	string s; s.resize(BUFSIZE);
	float val;
	while (fin >> s) {
		if (s[0] == '#') {
			fin.getline(&s[0], BUFSIZE, '\n');
			continue;
		}
		fin >> val;
		data[s] = val;
		fin.getline(&s[0], BUFSIZE, '\n');
	}
}

float ConfigParser::get(const std::string& s) {
	if (data.count(s) == 0)
		error_exit(ssprintf("Option %s not found in config file!\n", s.c_str()));
	return data[s];
}

bool CYLINDER;
bool TRANS;
bool CROP;
float FOCAL_LENGTH;
bool ESTIMATE_CAMERA;
bool STRAIGHTEN;
int MAX_OUTPUT_SIZE;
bool ORDERED_INPUT;
bool LAZY_READ;

int MULTIPASS_BA;
float LM_LAMBDA;

int SIFT_WORKING_SIZE;
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

float ORI_RADIUS;

int ORI_HIST_SMOOTH_COUNT;

int DESC_HIST_SCALE_FACTOR;
int DESC_INT_FACTOR;

float MATCH_REJECT_NEXT_RATIO;

int RANSAC_ITERATIONS;
double RANSAC_INLIER_THRES;
float INLIER_IN_MATCH_RATIO;
float INLIER_IN_POINTS_RATIO;

float SLOPE_PLAIN;

int MULTIBAND;

}
