// File: image.hh
// Date: Thu Apr 11 10:51:37 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <cstring>
#include <Magick++.h>
#include "color.hh"

class GreyImg;

class Img {
	protected:
		void init(int m_w, int m_h);

		void init_from_image(const Magick::Image& img);

	public:
		int w, h;
		Color* pixel;

		Img(){}

		Img(const Img& img):
			Img(img.w, img.h)
		{
			memcpy(pixel, img.pixel, w * h * sizeof(Color));
		}

		Img(int m_w, int m_h)
		{ init(m_w, m_h); }

		Img(const Magick::Image& img)
		{ init_from_image(img); }

		Img(const char* fname) {
			Magick::Image img(fname);
			init_from_image(img);
		}

		Img(const GreyImg& gr);

		~Img()
		{ delete[] pixel; }

		Img get_resized(real_t factor) const;

		const Color& get_pixel(int x, int y) const ;

		void set_pixel(int x, int y, const Color& c);
};

class GreyImg {
	protected:
		void init(int m_w, int m_h);

		void init_from_img(const Img& img);

	public:
		int w, h;
		real_t* pixel;

		GreyImg(){}

		GreyImg(const GreyImg& img):
			GreyImg(img.w, img.h)
		{
			memcpy(pixel, img.pixel, w * h * sizeof(real_t));
		}

		GreyImg(int m_w, int m_h)
		{ init(m_w, m_h); }

		GreyImg(const Img& img)
		{ init_from_img(img); }

		~GreyImg()
		{ delete[] pixel; }

		real_t get_pixel(int x, int y) const ;

		void set_pixel(int x, int y, real_t c);

};

