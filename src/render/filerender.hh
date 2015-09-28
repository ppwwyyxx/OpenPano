// File: filerender.hh
// Date: Sat May 04 12:53:17 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "render/render.hh"
#include <Magick++.h>
#include <cstring>

class FileRender : public RenderBase {
	private:
		Magick::Image img;
		std::string fname;
		Magick::Pixels *view;
		Magick::PixelPacket *pixels_ptr;

	public:
		FileRender(const ::Geometry& g, const char* m_fname):
			RenderBase(g),
			img(Magick::Geometry(g.w, g.h), "black"),
			 fname(m_fname) {
			img.type(Magick::TrueColorType);
			img.modifyImage();
			view = new Magick::Pixels(img);
			pixels_ptr = view->get(0, 0, g.w, g.h);
		}

		FileRender(int w, int h, const char* fname):
			FileRender(::Geometry(w, h), fname){}

		FileRender(std::shared_ptr<Img> img, const char* fname):
			FileRender(img->w, img->h, fname){
			write(img);
		}

		~FileRender() { delete view; }

		void finish() {
			view->sync();
			img.write(fname.c_str());
		}

	private:
		void _write(int x, int y, const ::Color &c) {
			if (c.get_max() < 0)
				// white background
				pixels_ptr[y * geo.w + x] = Magick::ColorRGB(1, 1, 1);
			else
				pixels_ptr[y * geo.w + x] = Magick::ColorRGB(c.x, c.y, c.z);
		}

};


