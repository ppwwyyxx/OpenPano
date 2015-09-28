// File: filerender.hh
// Date: Sat May 04 12:53:17 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "render/render.hh"
#include "lib/mat.h"
#define cimg_use_png
#define cimg_use_jpeg
#include "lib/CImg.h"
#include <cstring>
typedef cimg_library::CImg<float> Image;

class FileRender : public RenderBase {
	private:
		Image img;
		std::string fname;

	public:
		FileRender(const ::Geometry& g, const char* m_fname):
			RenderBase(g),
			img(g.w, g.h, 1, 3, 0),
			 fname(m_fname) {
		}

		FileRender(int w, int h, const char* fname):
			FileRender(::Geometry(w, h), fname){}

		FileRender(const Mat32f& img, const char* fname):
			FileRender(img.width(), img.height(), fname){
			write(img);
		}

		void finish() {
			img = img * 255.0;
			img.save(fname.c_str());
		}

	private:
		void _write(int x, int y, const ::Color &c) {
			if (c.get_max() < 0) {
				// white background
				img(x, y, 0) = 1;
				img(x, y, 1) = 1;
				img(x, y, 2) = 1;
			} else {
				img(x, y, 0) = c.x;
				img(x, y, 1) = c.y;
				img(x, y, 2) = c.z;
			}
		}

};


