// File: sift.cc
// Date: Sun Apr 14 20:23:05 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <omp.h>
#include "sift.hh"
#include "filter.hh"
using namespace std;


Octave::Octave(const shared_ptr<GreyImg>& img, int num_scale):
	nscale(num_scale){
	w = img->w, h = img->h;
	data = new shared_ptr<GreyImg>[nscale];
	data[0] = img;

	for (int i = 1; i < nscale; i ++) {
		real_t s = pow(SCALE_FACTOR, i);
		data[i] = Filter::GaussianBlur(data[i-1], s);
	}
}

Octave::~Octave()
{ delete[] data; }

ScaleSpace::ScaleSpace(const shared_ptr<Img>& img, int num_octave, int num_scale):
	noctave(num_octave), nscale(num_scale){
	octaves = new shared_ptr<Octave>[noctave];
	octaves[0] = shared_ptr<Octave>(new Octave(img, nscale));

#pragma omp parallel for schedule(dynamic)
	for (int i = 1; i < noctave; i ++) {
		Img now = img->get_resized(pow(SCALE_FACTOR, -i));
		octaves[i] = shared_ptr<Octave>(new Octave(make_shared<Img>(now), nscale));
	}
}

ScaleSpace::~ScaleSpace()
{ delete[] octaves; }

DOG::DOG(const shared_ptr<Octave>& o) {
	nscale = o->get_len();
	data = new shared_ptr<GreyImg>[nscale - 1];
	for (int i = 0; i < nscale - 1; i ++)
		data[i] = diff(o->get(i), o->get(i + 1));
}

DOG::~DOG()
{ delete[] data; }

// XXX what to do with diff?
shared_ptr<GreyImg> DOG::diff(const shared_ptr<GreyImg>& img1, const shared_ptr<GreyImg>& img2) {
	int w = img1->w, h = img1->h;
	m_assert(w == img2->w && h == img2->h);
	shared_ptr<GreyImg> ret(new GreyImg(w, h));
	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++) {
			real_t diff = fabs(img1->get_pixel(i, j) - img2->get_pixel(i, j));
			ret->set_pixel(i, j, diff);
		}
	return move(ret);
}

DOGSpace::DOGSpace(const shared_ptr<Img>& img, int num_octave, int num_scale):
	noctave(num_octave), nscale(num_scale) {
	origw = img->w, origh = img->h;
	dogs = new shared_ptr<DOG>[noctave];
	ScaleSpace ss(img, noctave, nscale);
	print_debug("finish constructing scalespace\n");
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < noctave; i ++)
		dogs[i] = shared_ptr<DOG>(new DOG(ss.octaves[i]));
}

DOGSpace::~DOGSpace()
{ delete[] dogs; }
