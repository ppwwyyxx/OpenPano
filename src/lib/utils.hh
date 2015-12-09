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
using namespace std;

#ifdef _WIN32
#define __attribute__(x)
#endif

string TERM_COLOR(int k);

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


template <typename T>
std::shared_ptr<T> create_auto_buf(size_t len, bool init_zero = false) {
	std::shared_ptr<T> ret(new T[len], [](T *ptr){delete []ptr;});
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
