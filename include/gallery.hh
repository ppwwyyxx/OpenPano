// File: gallery.hh
// Date: Fri Apr 19 23:38:37 2013 +0800
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


};
