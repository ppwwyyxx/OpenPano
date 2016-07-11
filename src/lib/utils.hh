// File: utils.hh
// Date: Tue Apr 23 11:31:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#pragma once

#include <cstdarg>
#include <cstdlib>
#include <string>
#include <cstring>
#include <sstream>
#include <memory>
#include <sys/stat.h>
#include <type_traits>

#ifdef _WIN32
#define __attribute__(x)
#endif

typedef double real_t;
const real_t EPS = 1e-6;
const real_t GEO_EPS_SQR = 1e-14;
const real_t GEO_EPS = 1e-7;
inline float sqr(float x) { return x * x; }

#define between(a, b, c) ((a >= b) && (a <= c - 1))
#define REP(x, y) for (typename std::remove_cv<typename std::remove_reference<decltype(y)>::type>::type x = 0; x < (y); x ++)
#define REPL(x, y, z) for (typename std::remove_cv<typename std::remove_reference<decltype(y)>::type>::type x = y; x < (z); x ++)
#define REPD(x, y, z) for (typename std::remove_cv<typename std::remove_reference<decltype(y)>::type>::type x = y; x >= (z); x --)

std::string TERM_COLOR(int k);

#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"
#define COLOR_RESET   "\x1b[0m"

void c_printf(const char* col, const char* fmt, ...);

void c_fprintf(const char* col, FILE* fp, const char* fmt, ...);

__attribute__ (( format( printf, 1, 2 ) ))
std::string ssprintf(const char *fmt, ...);

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

template <typename T>
inline void free_2d(T** ptr, int w) {
	if (ptr != nullptr)
		for (int i = 0; i < w; i ++)
			delete[] ptr[i];
	delete[] ptr;
}

template <typename T>
std::shared_ptr<T> create_auto_buf(size_t len, bool init_zero = false) {
	std::shared_ptr<T> ret(new T[len], std::default_delete<T[]>());
	if (init_zero)
		memset(ret.get(), 0, sizeof(T) * len);
	return ret;
}


inline bool exists_file(const char* name) {
	struct stat buffer;
	return stat(name, &buffer) == 0;
}

inline bool endswith(const char* str, const char* suffix) {
	if (!str || !suffix) return false;
	auto l1 = strlen(str), l2 = strlen(suffix);
	if (l2 > l1) return false;
	return strncmp(str + l1 - l2, suffix, l2) == 0;
}
