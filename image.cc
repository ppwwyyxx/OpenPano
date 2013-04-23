// File: image.cc
// Date: Tue Apr 23 11:08:06 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "image.hh"
#include "filter.hh"
#include "render/MImageRender.hh"
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
	REP(i, h) REP(j, w)
		set_pixel(i, j, c);
}

const ::Color& Img::get_pixel(int r, int c) const {
	m_assert(between(r, 0, h) && between(c, 0, w));
	::Color *dest = pixel + r * w + c;
	return *dest;
}

void Img::add_pixel(int y, int x, const ::Color& c)
{ set_pixel(y, x, get_pixel(y, x) + c); }

::Color Img::get_pixel(real_t y, real_t x) const {
	::Color ret = ::Color::BLACK;
	real_t dy = y - floor(y), dx = x - floor(x);
	ret += get_pixel((int)floor(y), (int)floor(x)) * ((1 - dy) * (1 - dx));
	ret += get_pixel((int)ceil(y), (int)floor(x)) * (dy * (1 - dx));
	ret += get_pixel((int)ceil(y), (int)ceil(x)) * (dy * dx);
	ret += get_pixel((int)floor(y), (int)ceil(x)) * ((1 - dy) * dx);
	return ret;
}

void Img::add_pixel(real_t y, real_t x, const ::Color& c) {
	int fly = floor(y), flx = floor(x);
	real_t dy = y - fly, dx = x - flx;
	add_pixel(fly, flx, c * ((1 - dy) * (1 - dx)));
	add_pixel(fly + 1, flx, c * (dy * (1 - dx)));
	add_pixel(fly + 1, flx + 1, c * (dy * dx));
	add_pixel(fly, flx + 1, c * ((1 - dy) * dx));
}

void Img::set_pixel(int r, int c, const ::Color& val) {
	m_assert(between(r, 0, h) && between(c, 0, w));
	::Color *dest = pixel + r * w + c;
	dest->x = val.x, dest->y = val.y, dest->z = val.z;
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

shared_ptr<Img> GreyImg::to_img() const {
	shared_ptr<Img> ret(new Img(w, h));
	REP(i, h) REP(j, w) {
		real_t grey = get_pixel(i, j);
		ret->set_pixel(i, j, ::Color(grey, grey, grey));
	}
	return ret;
}
