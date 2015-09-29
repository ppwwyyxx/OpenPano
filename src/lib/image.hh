// File: image.hh
// Date: Sun Dec 29 03:21:24 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <cstring>
#include <list>
#include <memory>
#include "color.hh"
#include "mat.h"

class GreyImg;

class Img : public std::enable_shared_from_this<Img> {
	public:
		Mat32f mat;
		int w, h;

		Img(const Img& img):mat(img.mat) {
			w = mat.width();
			h = mat.height();
		}

		Img(const Mat32f& mat):mat(mat) {
			w = mat.width();
			h = mat.height();
		}

		Img& operator=(const Img& img) {
			mat = img.mat;
			w = mat.width();
			h = mat.height();
			return *this;
		}

		Img(int m_w, int m_h):
			mat(m_h, m_w, 3) {
			w = mat.width();
			h = mat.height();
			}

		Img(const char* fname);

		Img(const GreyImg& gr);

		Img get_resized(real_t factor) const;

		Color get_pixel(int r, int c) const {
			return Color(mat.at(r, c, 0), mat.at(r, c, 1), mat.at(r, c, 2));
		}

		Color get_pixel(real_t y, real_t x) const;

		Color get_pixel(const Coor& w) const
		{ return get_pixel(w.y, w.x);}

		void set_pixel(int, int, const Color&);

		void fill(const Color& c);

		bool is_image_edge(real_t, real_t) const;

		Vec2D get_center() const { return Vec2D(mat.cols() / 2, mat.rows() / 2); }

		void crop();
};

class GreyImg {
	protected:
		void init(int m_w, int m_h);

		void init_from_img(const Img& img);

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

		~GreyImg()
		{ delete[] pixel; }

		std::shared_ptr<Img> to_img() const;

		real_t get_pixel(int x, int y) const;

		void set_pixel(int x, int y, real_t c);

};

typedef std::shared_ptr<Img> imgptr;
typedef std::shared_ptr<const Img> imgptrc;
