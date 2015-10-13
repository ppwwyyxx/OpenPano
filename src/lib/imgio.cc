//File: imgio.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "imgproc.hh"
#include <cstdlib>

#define cimg_display 0
#define cimg_use_png
#define cimg_use_jpeg
#include "CImg.h"

#include "lib/utils.hh"

using namespace cimg_library;
using namespace std;

Mat32f read_img(const char* fname) {
	if (not exists_file(fname))
		error_exit(ssprintf("File %s not exists!", fname));
	CImg<float> img(fname);
	m_assert(img.spectrum() == 3 || img.spectrum() == 1);
	img = img / 255.0;
	Mat32f mat(img.height(), img.width(), 3);
	if (img.spectrum() == 3) {
		REP(i, mat.rows())
			REP(j, mat.cols()) {
				mat.at(i, j, 0) = img(j, i, 0);
				mat.at(i, j, 1) = img(j, i, 1);
				mat.at(i, j, 2) = img(j, i, 2);
			}
	} else {
		REP(i, mat.rows())
			REP(j, mat.cols()) {
				mat.at(i, j, 0) = mat.at(i, j, 1) = mat.at(i, j, 2) = img(j, i);
			}
	}
	return mat;
}


void write_rgb(const char* fname, const Mat32f& mat) {
	m_assert(mat.channels() == 3);
	CImg<float> img(mat.cols(), mat.rows(), 1, 3);
	REP(i, mat.rows())
		REP(j, mat.cols()) {
			// use white background. Color::NO turns to 1
			img(j, i, 0) = (mat.at(i, j, 0) < 0 ? 1 : mat.at(i, j, 0)) * 255;
			img(j, i, 1) = (mat.at(i, j, 1) < 0 ? 1 : mat.at(i, j, 1)) * 255;
			img(j, i, 2) = (mat.at(i, j, 2) < 0 ? 1 : mat.at(i, j, 2)) * 255;
		}
	img.save(fname);
}

