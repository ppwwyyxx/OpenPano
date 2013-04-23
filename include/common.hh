// File: common.hh
// Date: Tue Apr 23 12:27:49 2013 +0800
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
inline real_t sqr(real_t x) { return x * x; }


const int NUM_OCTAVE = 4;
const int NUM_SCALE = 6;
const real_t SCALE_FACTOR = sqrt(2);

const real_t GAUSS_SIGMA = SCALE_FACTOR;
const int GAUSS_WINDOW_FACTOR = 6;		// before: 4

const real_t JUDGE_EXTREMA_DIFF_THRES = +2e-7;
const real_t CONTRAST_THRES = 3e-2;		// before: 7e-3
const real_t PRE_COLOR_THRES = 2e-4;		// before : 3e-5
const real_t EDGE_RATIO = 10;		// lowe

const int CALC_OFFSET_DEPTH = 4;
const real_t OFFSET_THRES = 0.6;

const real_t ORI_WINDOW_FACTOR = 1.5;		// lowe
const real_t ORI_RADIUS = 3 * ORI_WINDOW_FACTOR;

const int ORT_HIST_BIN_NUM = 36;		// lowe
const int ORT_HIST_SMOOTH_COUNT = 2;
const real_t ORT_HIST_PEAK_RATIO = 0.8;		// lowe

const int DESC_HIST_WIDTH = 4;
const int DESC_HIST_REAL_WIDTH = 3;
const int DESC_HIST_BIN_NUM = 8;
const real_t DESC_NORM_THRESH = 0.2;		// lowe
const int DESC_INT_FACTOR = 512;
const int DESC_LEN = 128;

const real_t MATCH_REJECT_NEXT_RATIO = 0.8; // lowe
const int MATCH_MIN_SIZE = 30;

const int AFFINE_REQUIRED_MATCH = 4;
const int RANSAC_ITERATIONS = 50;
const real_t RANSAC_INLIER_THRES = 5;

#define between(a, b, c) ((a >= b) && (a <= c - 1))
#define REP(x, y) for (int x = 0; x < (y); x ++)
#define REPL(x, y, z) for (int x = y; x < (z); x ++)

#define toCoor(a) Coor(a.x, a.y)

template <typename T>
inline void free_2d(T** ptr, int w) {
	if (ptr != nullptr)
		for (int i = 0; i < w; i ++)
			delete[] ptr[i];
	delete[] ptr;
}

template<typename T>
bool update_min(T &dest, const T &val) {
	if (val < dest) {
		dest = val; return true;
	}
	return false;
}

template<typename T>
bool update_max(T &dest, const T &val) {
	if (dest < val) {
		dest = val; return true;
	}
	return false;
}

