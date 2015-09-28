//File: imgproc.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "imgproc.hh"

#include <vector>

#define cimg_use_png
#define cimg_use_jpeg
#include "CImg.h"

#include "debugutils.hh"
#include "common.hh"

using namespace cimg_library;
using namespace std;

Mat32f read_rgb(const char* fname) {
	CImg<float> img(fname);
	m_assert(img.spectrum() == 3);
	img = img / 255.0;
	Mat32f mat(img.height(), img.width(), 3);
	REP(i, mat.rows())
		REP(j, mat.cols()) {
			mat.at(i, j, 0) = img(j, i, 0);
			mat.at(i, j, 1) = img(j, i, 1);
			mat.at(i, j, 2) = img(j, i, 2);
		}
	return mat;
}

namespace {

void resize_bilinear(const Mat32f &src, Mat32f &dst) {
    vector<int> tabsx(dst.rows());
    vector<int> tabsy(dst.cols());
    vector<float> tabrx(dst.rows());
    vector<float> tabry(dst.cols());

    const float fx = (float)(dst.rows()) / src.rows();
    const float fy = (float)(dst.cols()) / src.cols();
    const float ifx = 1.f / fx;
    const float ify = 1.f / fy;
    for (int dx = 0; dx < dst.rows(); ++dx) {
        float rx = (dx+0.5f) * ifx - 0.5f;
        int sx = floor(rx);
        rx -= sx;
        if (sx < 0) {
            sx = rx = 0;
        } else if (sx + 1 >= src.rows()) {
            sx = src.rows() - 2;
            rx = 1;
        }
        tabsx[dx] = sx;
        tabrx[dx] = rx;
    }
    for (int dy = 0; dy < dst.cols(); ++dy) {
        float ry = (dy+0.5f) * ify - 0.5f;
        int sy = floor(ry);
        ry -= sy;
        if (sy < 0) {
            sy = ry = 0;
            ry = 0;
        } else if (sy + 1 >= src.cols()) {
            sy = src.cols() - 2;
            ry = 1;
        }
        tabsy[dy] = sy;
        tabry[dy] = ry;
    }

    const int ch = src.channels();
    for (int dx = 0; dx < dst.rows(); ++dx) {
        const float *p0 = src.ptr(tabsx[dx]+0);
        const float *p1 = src.ptr(tabsx[dx]+1);
        float *pdst = dst.ptr(dx);
        float rx = tabrx[dx], irx = 1.0f - rx;
        for (int dy = 0; dy < dst.cols(); ++dy) {
            float *pcdst = pdst + dy*ch;
            const float *pc00 = p0 + (tabsy[dy]+0)*ch;
            const float *pc01 = p0 + (tabsy[dy]+1)*ch;
            const float *pc10 = p1 + (tabsy[dy]+0)*ch;
            const float *pc11 = p1 + (tabsy[dy]+1)*ch;
            float ry = tabry[dy], iry = 1.0f - ry;
            for (int c = 0; c < ch; ++c) {
                float res = rx * (pc11[c]*ry + pc10[c]*iry)
									+ irx * (pc01[c]*ry + pc00[c]*iry);
                pcdst[c] = res;
            }
        }
    }
}
}	// namespace

template <>
void resize<float>(const Mat32f &src, Mat32f &dst) {
    m_assert(src.rows() > 1);
    m_assert(src.cols() > 1);
    m_assert(dst.rows() > 1);
    m_assert(dst.cols() > 1);
    m_assert(src.channels() == dst.channels());
    m_assert(src.channels() == 1 || src.channels() == 3);
		return resize_bilinear(src, dst);
}
