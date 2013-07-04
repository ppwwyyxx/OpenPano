// File: image.cc
// Date: Thu Jul 04 11:43:20 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "image.hh"
#include "filter.hh"
#include "render/MImageRender.hh"
#include "cylinder.hh"
using namespace std;
using namespace Magick;

void Img::init(int m_w, int m_h) {
	w = m_w, h = m_h;
	pixel = new ::Color[w * h];
}

void GreyImg::init(int m_w, int m_h) {
	w = m_w, h = m_h;
	pixel = new real_t[w * h];
}

void Img::init_from_image(const Image& img) {
	Magick::Geometry size = img.size();
	init(size.width(), size.height());

	::Color *dest = pixel;

	const PixelPacket* src = img.getConstPixels(0, 0, w, h);
	REP(i, h) REP(j, w) {
		dest->x = double(src->red) / QuantumRange;
		dest->y = double(src->green) / QuantumRange;
		dest->z = double(src->blue) / QuantumRange;
		dest ++;
		src ++;
	}
}

Img::Img(const GreyImg& gr) {
	init(gr.w, gr.h);
	REP(i, h) REP(j, w) {
		real_t grey = gr.get_pixel(i, j);
		set_pixel(i, j, ::Color(grey, grey, grey));
	}
}

Img Img::get_resized(real_t factor) const {
	int neww = ceil(w * factor),
		newh = ceil(h * factor);
	Image img = MImg(shared_from_this()).get_img();
	img.resize(Magick::Geometry(neww, newh));
	return move(img);
}

void Img::fill(const ::Color& c) {
#pragma omp parallel for schedule(static)
	REP(i, h) REP(j, w) set_pixel(i, j, c);
}

const ::Color& Img::get_pixel(int r, int c) const {
	m_assert(between(r, 0, h) && between(c, 0, w));
	::Color *dest = pixel + r * w + c;
	return *dest;
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
	::Color *dest = pixel + r * w + c;
	dest->x = val.x, dest->y = val.y, dest->z = val.z;
}

void Img::crop() {
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
