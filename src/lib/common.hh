// File: common.hh
// Date: Fri May 03 04:51:14 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <iostream>
#include <limits>

#include "debugutils.hh"

typedef double real_t;
const real_t EPS = 1e-6;
const real_t SEPS = std::numeric_limits<real_t>::epsilon();
inline float sqr(float x) { return x * x; }

#define between(a, b, c) ((a >= b) && (a <= c - 1))
#define REP(x, y) for (auto x = decltype(y){0}; x < (y); x ++)
#define REPL(x, y, z) for (auto x = decltype(y){y}; x < (z); x ++)
#define REPD(x, y, z) for (auto x = decltype(y){y}; x >= (z); x --)

#define toCoor(a) Coor((a).x, (a).y)

template <typename T>
inline void free_2d(T** ptr, int w) {
	if (ptr != nullptr)
		for (int i = 0; i < w; i ++)
			delete[] ptr[i];
	delete[] ptr;
}

template<typename T>
inline bool update_min(T &dest, const T &val) {
	if (val < dest) {
		dest = val; return true;
	}
	return false;
}

template<typename T>
inline bool update_max(T &dest, const T &val) {
	if (dest < val) {
		dest = val; return true;
	}
	return false;
}

