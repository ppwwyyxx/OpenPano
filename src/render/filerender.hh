// File: filerender.hh
// Date: Sat May 04 12:53:17 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "render/render.hh"
#include "lib/mat.h"
#include "lib/imgproc.hh"
#include <cstring>

class FileRender : public RenderBase {
	private:
		Mat32f img;
		std::string fname;

	public:
		FileRender(const ::Geometry& g, const char* fname):
			RenderBase(g),
			img(g.h, g.w, 3),
			fname(fname) {
			fill(img, Color::BLACK);
		}

		FileRender(int w, int h, const char* fname):
			FileRender(::Geometry(w, h), fname){}

		FileRender(const Mat32f& m, const char* fname):
			FileRender(m.width(), m.height(), fname) {
			write(m);
		}

		void finish() {
			write_rgb(fname.c_str(), img);
		}

	private:
		void _write(int x, int y, const ::Color &c) {
			if (c.get_max() < 0) {
				// white background for COLOR::NO
				img.at(y, x, 0) = 1;
				img.at(y, x, 1) = 1;
				img.at(y, x, 2) = 1;
			} else {
				img.at(y, x, 0) = c.x;
				img.at(y, x, 1) = c.y;
				img.at(y, x, 2) = c.z;
			}
		}

};


