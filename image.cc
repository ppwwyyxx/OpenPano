// File: image.cc
// Date: Thu Apr 11 11:01:51 2013 +0800
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
	for (int y = 0; y < h; y ++)
		for (int x = 0; x < w; x ++)
		{
			dest->x = double(src->red) / QuantumRange;
			dest->y = double(src->green) / QuantumRange;
			dest->z = double(src->blue) / QuantumRange;
			dest ++;
			src ++;
		}
}

Img::Img(const GreyImg& gr) {
	init(gr.w, gr.h);
	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++) {
			real_t grey = gr.get_pixel(i, j);
			set_pixel(i, j, ::Color(grey, grey, grey));
		}
}

Img Img::get_resized(real_t factor) const {
	int neww = ceil(w * factor),
		newh = ceil(h * factor);
	Image img = MImg(this).get_img();
	img.resize(Magick::Geometry(neww, newh));
	return Img(img);
}


const ::Color& Img::get_pixel(int i, int j) const {
	::Color *dest = pixel + i * w + j;
	return *dest;

}

void Img::set_pixel(int i, int j, const ::Color& c) {
	::Color *dest = pixel + i * w + j;
	dest->x = c.x, dest->y = c.y, dest->z = c.z;
}

real_t GreyImg::get_pixel(int x, int y) const
{ return *(pixel + x * w + y); }

void GreyImg::set_pixel(int x, int y, real_t c)
{ *(pixel + x * w + y) = c; }

void GreyImg::init_from_img(const Img& img) {
	init(img.w, img.h);
	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++) {
			set_pixel(i, j, Filter::to_grey(img.get_pixel(i, j)));
		}
}
