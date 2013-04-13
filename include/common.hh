// File: common.hh
// Date: Sun Apr 14 01:19:59 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <cmath>
#include <vector>
#include <iostream>
#include <utility>
#include <limits>

#include "debugutils.hh"

typedef double real_t;
const real_t EPS = 1e-6;

const int NUM_OCTAVE = 4;
const int NUM_SCALE = 5;
const int SIFT_INTERPOLATION_DEPTH = 5;
const real_t JUDGE_EXTREMA_DIFF_THRES = 2e-7;
const real_t CONTRAST_THRES = 0.025;
const real_t PRE_COLOR_THRES = 3e-4;
const real_t SCALE_FACTOR = sqrt(2);
const real_t EDGE_RATIO = 10;

template<typename T>
bool update_min(T &dest, const T &val) {
	if (val < dest) {
		dest = val; return true;
	}
	return false;
}

inline real_t sqr(real_t x)
{ return x * x; }

template<typename T>
bool update_max(T &dest, const T &val) {
	if (dest < val) {
		dest = val; return true;
	}
	return false;
}

