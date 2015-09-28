// File: image.cc
// Date: Sun Dec 29 03:20:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "image.hh"
#include "imgproc.hh"
#include "filter.hh"
#include "cylinder.hh"
#define cimg_use_png
#define cimg_use_jpeg
#include "CImg.h"
using namespace cimg_library;

#include "lib/utils.hh"

#include <list>
using namespace std;

typedef cimg_library::CImg<float> Image;


static Image get_cimg(const Img& img) {
	Image	ret(img.w, img.h, 1, 3, 0);
	REP(i, img.mat.rows())
		REP(j, img.mat.cols()) {
			ret(j, i, 0) = img.mat.at(i, j, 0);
			ret(j, i, 1) = img.mat.at(i, j, 1);
			ret(j, i, 2) = img.mat.at(i, j, 2);
		}
	return ret;
}

void GreyImg::init(int m_w, int m_h) {
	w = m_w, h = m_h;
	pixel = new real_t[w * h];
}

Img::Img(const char* fname) {
	mat = read_rgb(fname);
	w = mat.width();
	h = mat.height();
}

Img::Img(const GreyImg& gr):mat(gr.w, gr.h, 3) {
	w = mat.width(), h = mat.height();
	REP(i, h) REP(j, w) {
		real_t grey = gr.get_pixel(i, j);
		set_pixel(i, j, ::Color(grey, grey, grey));
	}
}

Img Img::get_resized(real_t factor) const {
	int neww = ceil(mat.width() * factor), newh = ceil(h * factor);
	Img ret(neww, newh);
	resize(mat, ret.mat);
	return ret;
}

void Img::fill(const ::Color& c) {
#pragma omp parallel for schedule(static)
	REP(i, h) REP(j, w) set_pixel(i, j, c);
}

::Color Img::get_pixel(real_t y, real_t x) const {
	::Color ret = ::Color::BLACK;
	real_t dy = y - floor(y), dx = x - floor(x);
	ret += get_pixel((int)floor(y), (int)floor(x)) * ((1 - dy) * (1 - dx));
	ret += get_pixel((int)ceil(y), (int)floor(x)) * (dy * (1 - dx));
	ret += get_pixel((int)ceil(y), (int)ceil(x)) * (dy * dx);
	ret += get_pixel((int)floor(y), (int)ceil(x)) * ((1 - dy) * dx);
	return ret;
}

bool Img::is_image_edge(real_t y, real_t x) const {		// judge Color::NO
	if (!between(x, 0, w) || !between(y, 0, h)) return true;
	if (get_pixel((int)floor(y), (int)floor(x)).get_min() < 0) return true;
	if (get_pixel((int)ceil(y), (int)floor(x)).get_min() < 0) return true;
	if (get_pixel((int)ceil(y), (int)ceil(x)).get_min() < 0) return true;
	if (get_pixel((int)floor(y), (int)ceil(x)).get_min() < 0) return true;
	return false;
}

void Img::set_pixel(int r, int c, const ::Color& val) {
	m_assert(between(r, 0, h) && between(c, 0, w));
	mat.at(r, c, 0) = val.x;
	mat.at(r, c, 1) = val.y;
	mat.at(r, c, 2) = val.z;
}

void Img::crop() {
	int w = mat.width(), h = mat.height();
	int height[w], left[w], right[w];
	int maxarea = 0;
	int ll = 0, rr = 0, hh = 0, nl = 0;
	memset(height, 0, sizeof(height));
	REP(line, h) {
		REP(k, w)
			height[k] = get_pixel(line, k).get_max() < 0 ? 0 : height[k] + 1;	// judge Color::NO

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
	Img ret(rr - ll + 1, hh);
	int offsetx = ll, offsety = nl - hh + 1;
	REP(i, ret.h) REP(j, ret.w)
		ret.set_pixel(i, j, get_pixel(i + offsety, j + offsetx));
	*this = move(ret);
}

real_t GreyImg::get_pixel(int r, int c) const {
	m_assert(between(r, 0, h) && between(c, 0, w));
	return *(pixel + r * w + c);
}

void GreyImg::set_pixel(int r, int c, real_t val) {
	m_assert(between(r, 0, h) && between(c, 0, w));
	*(pixel + r * w + c) = val;
}

void GreyImg::init_from_img(const Img& img) {
	init(img.w, img.h);
	REP(i, h) REP(j, w)
		set_pixel(i, j, Filter::to_grey(img.get_pixel(i, j)));
}

imgptr GreyImg::to_img() const {
	imgptr ret = make_shared<Img>(w, h);
	REP(i, h) REP(j, w) {
		real_t grey = get_pixel(i, j);
		ret->set_pixel(i, j, ::Color(grey, grey, grey));
	}
	return ret;
}

// XXX
static void init_from_image(Img* img, const Image& image) {
	Mat32f mat(image.height(), image.width(), 3);
	REP(i, mat.rows())
		REP(j, mat.cols()) {
			mat.at(i, j, 0) = image(j, i, 0);
			mat.at(i, j, 1) = image(j, i, 1);
			mat.at(i, j, 2) = image(j, i, 2);
		}
	img->mat = mat;
	img->w = mat.width();
	img->h = mat.height();
}

Img hconcat(const std::list<Img>& imglist) {
	CImgList<float> imgs;
	for (auto& i : imglist)
		imgs.insert(get_cimg(i));
	Image img = imgs.get_append('x');
	Img ret(img.width(), img.height());
	init_from_image(&ret, img);
	return ret;
}
