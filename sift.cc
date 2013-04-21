// File: sift.cc
// Date: Sun Apr 21 20:19:45 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "sift.hh"
#include "filter.hh"
using namespace std;

#define between(a, b, c) ((a >= b) && (a < c))

Octave::Octave(const shared_ptr<GreyImg>& img, int num_scale):
	nscale(num_scale){
	w = img->w, h = img->h;
	data = new shared_ptr<GreyImg>[nscale];
	mag = new shared_ptr<GreyImg>[nscale];
	ort = new shared_ptr<GreyImg>[nscale];
	data[0] = img;

	for (int i = 1; i < nscale; i ++) {
		real_t s = pow(SCALE_FACTOR, i - 1) * GAUSS_SIGMA;
		data[i] = Filter::GaussianBlur(data[i-1], s);	// sigma needs a better one
		cal_mag_ort(i);
	}
}

void Octave::cal_mag_ort(int i) {
	shared_ptr<GreyImg> orig = data[i];
	int w = orig->w, h = orig->h;
	mag[i] = shared_ptr<GreyImg>(new GreyImg(w, h));
	ort[i] = shared_ptr<GreyImg>(new GreyImg(w, h));
	for (int x = 0; x < w; x ++)
		for (int y = 0; y < h; y ++) {
			if (between(x, 1, w - 1) && between(y, 1, h - 1)) {
				real_t dy = orig->get_pixel(y + 1, x) - orig->get_pixel(y - 1, x),
					   dx = orig->get_pixel(y, x + 1) - orig->get_pixel(y, x - 1);
				mag[i]->set_pixel(y, x, hypot(dx, dy));
				if (dx == 0 && dy == 0)
					ort[i]->set_pixel(y, x, M_PI);
				else
					ort[i]->set_pixel(y, x, atan2(dy, dx) + M_PI);
			} else {
				mag[i]->set_pixel(y, x, 0);
				ort[i]->set_pixel(y, x, M_PI);
			}
		}
}

Octave::~Octave() {
	delete[] data;
	delete[] mag;
	delete[] ort;
}

ScaleSpace::ScaleSpace(const shared_ptr<Img>& img, int num_octave, int num_scale):
	noctave(num_octave), nscale(num_scale){
	origw = img->w, origh = img->h;
	octaves = new shared_ptr<Octave>[noctave];

#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < noctave; i ++) {
		if (!i)
			octaves[i] = shared_ptr<Octave>(new Octave(img, nscale));
		else {
			Img now = img->get_resized(pow(SCALE_FACTOR, -i));		// use pointer to save memory!
			octaves[i] = shared_ptr<Octave>(new Octave(make_shared<Img>(now), nscale));
		}
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

DOGSpace::DOGSpace(ScaleSpace& ss):
	noctave(ss.noctave), nscale(ss.nscale) {
	origw = ss.origw, origh = ss.origh;
	dogs = new shared_ptr<DOG>[noctave];

#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < noctave; i ++)
		dogs[i] = shared_ptr<DOG>(new DOG(ss.octaves[i]));
}

DOGSpace::~DOGSpace()
{ delete[] dogs; }

#undef between
