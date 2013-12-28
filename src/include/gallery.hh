// File: gallery.hh
// Date: Sat Dec 28 18:15:00 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "image.hh"
#include <Magick++.h>
#include <list>

class Gallery {
	protected:
		Magick::Image img;

	public:
		Gallery(std::list<Magick::Image>& List) {
			Magick::appendImages(&img, List.begin(), List.end(), false);
		}

		void save(const char* fname)
		{ img.write(fname); }

		const Magick::Image& get() const
		{ return img; }


};
