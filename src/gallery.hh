// File: gallery.hh
// Date: Sat Dec 28 18:15:00 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "lib/image.hh"
#include "lib/CImg.h"
#include <list>

class Gallery {
	protected:
		Image img;

	public:
		Gallery(std::list<Image>& List) {
			cimg_library::CImgList<float> imgs;
			for (auto& i : List)
				imgs.insert(i);
			img = imgs.get_append('x');
		}

		void save(const char* fname)
		{ img.save(fname); }

		const Image& get() const
		{ return img; }


};
