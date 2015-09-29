// File: utils.cc
// Date: Tue Apr 23 11:33:17 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "utils.hh"

string TERM_COLOR(int k) {
	// k = 0 ~ 4
	ostringstream ss;
	ss << "\x1b[3" << k + 2 << "m";
	return ss.str();
}

void c_printf(const char* col, const char* fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	printf("%s", col);
	vprintf(fmt, ap);
	printf(COLOR_RESET);
	va_end(ap);
}

void c_fprintf(const char* col, FILE* fp, const char* fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	fprintf(fp, "%s", col);
	vfprintf(fp, fmt, ap);
	fprintf(fp, COLOR_RESET);
	va_end(ap);
}

