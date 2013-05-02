// File: gallery.hh
// Date: Sat Apr 20 00:27:58 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "image.hh"
#include <Magick++.h>
#include <list>

class Gallery {
	protected:
		Magick::Image img;

	public:
		Gallery(std::list<Magick::Image>&);

		void save(const char* fname)
		{ img.write(fname); }

		const Magick::Image& get() const
		{ return img; }


};
