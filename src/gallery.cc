// File: gallery.cc
// Date: Sat Apr 27 15:49:12 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "gallery.hh"
using namespace std;
using namespace Magick;

Gallery::Gallery(list<Image>& List) {
	appendImages(&img, List.begin(), List.end(), false);
}

