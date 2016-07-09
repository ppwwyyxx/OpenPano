// File: utils.cc
// Date: Tue Apr 23 11:33:17 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "utils.hh"

std::string TERM_COLOR(int k) {
	// k = 0 ~ 4
	std::ostringstream ss;
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


std::string ssprintf(const char *fmt, ...) {
	int size = 100;
	char *p = (char *)malloc(size);

	va_list ap;

	std::string ret;

	while (true) {
		va_start(ap, fmt);
		int n = vsnprintf(p, size, fmt, ap);
		va_end(ap);

		if (n < 0) {
			free(p);
			return "";
		}

		if (n < size) {
			ret = p;
			free(p);
			return ret;
		}

		size = n + 1;

		char *np = (char *)realloc(p, size);
		if (np == nullptr) {
			free(p);
			return "";
		} else
			p = np;
	}
}
