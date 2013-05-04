// File: image.hh
// Date: Sat May 04 12:57:00 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <cstring>
#include <memory>
#include <Magick++.h>
#include "color.hh"

class GreyImg;

class Img : public std::enable_shared_from_this<Img> {
	protected:
		void init(int m_w, int m_h);

		void init_from_image(const Magick::Image& img);

	public:
		int w = 0, h = 0;
		Color* pixel;

		Img(const Img& img):
			Img(img.w, img.h)
		{ memcpy(pixel, img.pixel, w * h * sizeof(Color)); }

		Img(Img&& img) {
			w = img.w, h = img.h;
			pixel = img.pixel;
			img.pixel = nullptr;
		}

		Img & operator = (Img && r) {
			m_assert(this != &r);
			delete[] pixel;
			pixel = r.pixel;
			w = r.w, h = r.h;
			r.pixel = nullptr;
			return *this;
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

		const Color& get_pixel(int, int) const;

		Color get_pixel(real_t, real_t) const;

		const Color& get_pixel(const Coor& w) const
		{ return get_pixel(w.y, w.x);}

		Color get_pixel(const Vec2D& w) const
		{ return get_pixel(w.y, w.x);}

		void set_pixel(int, int, const Color&);

		void fill(const Color& c);

		bool is_image_edge(real_t, real_t) const;

		Vec2D get_center() const { return Vec2D(w / 2, h / 2); }

		void crop();
};

class GreyImg {
	protected:
		void init(int m_w, int m_h);

		void init_from_img(const Img& img);

		void init_from_image(const Magick::Image img);

	public:
		int w, h;
		real_t* pixel;

		GreyImg(const GreyImg& img):
			GreyImg(img.w, img.h)
		{ memcpy(pixel, img.pixel, w * h * sizeof(real_t)); }

		GreyImg(int m_w, int m_h)
		{ init(m_w, m_h); }

		GreyImg(const Img& img)
		{ init_from_img(img); }

		GreyImg(const Magick::Image& img)
		{ init_from_image(img); }

		~GreyImg()
		{ delete[] pixel; }

		std::shared_ptr<Img> to_img() const;

		real_t get_pixel(int x, int y) const;

		void set_pixel(int x, int y, real_t c);

};


typedef std::shared_ptr<Img> imgptr;
typedef std::shared_ptr<const Img> imgptrc;
