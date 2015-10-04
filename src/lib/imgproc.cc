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
	// TODO handle grey img
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


void write_rgb(const char* fname, const Mat32f& mat) {
	m_assert(mat.channels() == 3);
	CImg<float> img(mat.cols(), mat.rows(), 1, 3);
	REP(i, mat.rows())
		REP(j, mat.cols()) {
			// use black background. Color::NO turns to 0
			img(j, i, 0) = max(mat.at(i, j, 0), 0.f) * 255;
			img(j, i, 1) = max(mat.at(i, j, 1), 0.f) * 255;
			img(j, i, 2) = max(mat.at(i, j, 2), 0.f) * 255;
		}
	img.save(fname);
}


// hconcat using largest height and zero padding
Mat32f hconcat(const list<Mat32f>& mats) {
	int wsum = 0;
	int hmax = 0;
	for (auto& m: mats) {
		wsum += m.width();
		update_max(hmax, m.height());
	}
	int channel = mats.front().channels();

	Mat32f ret(hmax, wsum, channel);
	fill(ret, Color::BLACK);
	wsum = 0;
	for (auto & m : mats) {
		m_assert(m.channels() == channel);
		REP(i, m.height()) {
			const float* src = m.ptr(i);
			float* dst = ret.ptr(i, wsum);
			memcpy(dst, src, m.width() * channel * sizeof(float));
		}
		wsum += m.width();
	}
	return ret;
}

Color interpolate(const Mat32f& mat, float r, float c) {
	m_assert(mat.channels() == 3);
	Color ret = Color::BLACK;
	int fr = floor(r), fc =  floor(c);
	float dy = r - fr, dx = c - fc;
	const float* p = mat.ptr(fr, fc);
	ret += Color(p) * ((1 - dy) * (1 - dx));
	p = mat.ptr(fr + 1, fc);
	ret += Color(p) * (dy * (1 - dx));
	p = mat.ptr(fr + 1, fc + 1);
	ret += Color(p) * (dy * dx);
	p = mat.ptr(fr, fc + 1);
	ret += Color(p[0], p[1], p[2]) * ((1 - dy) * dx);
	return ret;
}

bool is_edge_color(const Mat32f& mat, float y, float x) {
	int w = mat.width(), h = mat.height();
	if (!between(x, 0, w) || !between(y, 0, h)) return true;
	m_assert(mat.channels() == 3);
	int fx = floor(x), fy = floor(y);

	const float* ptr = mat.ptr(fy, fx);
	REP(i, 6) if (ptr[i] < 0) return true;
	ptr = mat.ptr(fy + 1, fx);
	REP(i, 6) if (ptr[i] < 0) return true;
	return false;
}

void fill(Mat32f& mat, const Color& c) {
	float* ptr = mat.ptr();
	int n = mat.pixels();
	REP(i, n) {
		c.write_to(ptr);
		ptr += 3;
	}
}

Mat32f crop(const Mat32f& mat) {
	int w = mat.width(), h = mat.height();
	int height[w], left[w], right[w];
	int maxarea = 0;
	int ll = 0, rr = 0, hh = 0, nl = 0;
	memset(height, 0, sizeof(height));
	REP(line, h) {
		REP(k, w) {
			const float* p = mat.ptr(line, k);
			float m = max(max(p[0], p[1]), p[2]);
			height[k] = m < 0 ? 0 : height[k] + 1;	// judge Color::NO
		}

		REP(k, w) {
			left[k] = k;
			while (left[k] > 0 && height[k] <= height[left[k] - 1])
				left[k] = left[left[k] - 1];
		}
		REPD(k, w - 1, 0) {
			right[k] = k;
			while (right[k] < w - 1 && height[k] <= height[right[k] + 1])
				right[k] = right[right[k] + 1];
		}
		REP(k, w)
			if (update_max(maxarea, (right[k] - left[k] + 1) * height[k]))
				ll = left[k], rr = right[k], hh = height[k], nl = line;
	}
	Mat32f ret(hh, rr - ll + 1, 3);
	int offsetx = ll, offsety = nl - hh + 1;
	REP(i, ret.height()) {
		float* dst = ret.ptr(i, 0);
		const float* src = mat.ptr(i + offsety, offsetx);
		memcpy(dst, src, 3 * ret.width() * sizeof(float));
	}
	return ret;
}

Mat32f rgb2grey(const Mat32f& mat) {
	Mat32f ret(mat.height(), mat.width(), 1);
	const float* src = mat.ptr();
	float* dst = ret.ptr();
	int n = mat.pixels();
	int idx = 0;
	for (int i = 0; i < n; ++i) {
		dst[i] = (src[idx] + src[idx+1] + src[idx+2]) / 3.f;
		idx += 3;
	}
	return ret;
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
