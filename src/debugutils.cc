// File: debugutils.cc
// Date: Wed May 01 10:12:46 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <cstdarg>
#include <map>
using namespace std;

#include "utils.hh"

void __m_assert_check__(bool val, const char *expr, const char *file, const char *func, int line) {
	if (val)
		return;
	c_fprintf(COLOR_RED, stderr, "assertion \"%s\" failed, in %s, (%s:%d)\n",
			expr, func, file, line);
	abort();
}


void __print_debug__(const char *file, const char *func, int line, const char *fmt, ...) {
	static map<int, string> colormap;
	if (! colormap[line].length()) {
		static int color = 0;
		colormap[line] = TERM_COLOR(color);
		color = (color + 1) % 5;
	}

	char *fbase = basename(strdupa(file));
	c_fprintf(colormap[line].c_str(), stderr, "[%s@%s:%d] ", func, fbase, line);

	va_list ap;
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
}


void error_exit(const char *msg) {
	c_fprintf(COLOR_RED, stderr, "error: %s\n", msg);
	abort();
}
