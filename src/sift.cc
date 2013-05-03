// File: sift.cc
// Date: Sat May 04 01:27:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "config.hh"
#include "sift.hh"
#include "utils.hh"
#include "filter.hh"
using namespace std;

Octave::Octave(const shared_ptr<GreyImg>& img, int num_scale):
	nscale(num_scale){
	w = img->w, h = img->h;
	data = new shared_ptr<GreyImg>[nscale];
	mag = new shared_ptr<GreyImg>[nscale];
	ort = new shared_ptr<GreyImg>[nscale];
	data[0] = img;

	Filter blurer(nscale, GAUSS_SIGMA, SCALE_FACTOR);
	m_assert(blurer.gcache.size() == nscale - 1);
	for (int i = 1; i < nscale; i ++) {
		data[i] = blurer.GaussianBlur(data[0], i);	// sigma needs a better one
		cal_mag_ort(i);
	}
}

void Octave::cal_mag_ort(int i) {
	shared_ptr<GreyImg> orig = data[i];
	int w = orig->w, h = orig->h;
	mag[i] = shared_ptr<GreyImg>(new GreyImg(w, h));
	ort[i] = shared_ptr<GreyImg>(new GreyImg(w, h));
	REP(x, w) REP(y, h) {
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

		HWTimer timer;
// #pragma omp parallel for schedule(dynamic)
		REP(i, noctave) {
			if (!i)
				octaves[i] = shared_ptr<Octave>(new Octave(img, nscale));
			else {
				shared_ptr<Img> resized(new Img(img->get_resized(pow(SCALE_FACTOR, -i))));
				octaves[i] = shared_ptr<Octave>(new Octave(resized, nscale));
			}
		}
		print_debug("building scale space takes %lf\n", timer.get_sec());
	}

ScaleSpace::~ScaleSpace()
{ delete[] octaves; }

DOG::DOG(const shared_ptr<Octave>& o) {
	nscale = o->get_len();
	data = new shared_ptr<GreyImg>[nscale - 1];
	REP(i, nscale - 1)
		data[i] = diff(o->get(i), o->get(i + 1));
}

DOG::~DOG()
{ delete[] data; }

shared_ptr<GreyImg> DOG::diff(const shared_ptr<GreyImg>& img1, const shared_ptr<GreyImg>& img2) {
	int w = img1->w, h = img1->h;
	m_assert(w == img2->w && h == img2->h);
	shared_ptr<GreyImg> ret(new GreyImg(w, h));
	REP(i, h) REP(j, w) {
		real_t diff = fabs(img1->get_pixel(i, j) - img2->get_pixel(i, j));
		ret->set_pixel(i, j, diff);
	}
	return ret;
}

DOGSpace::DOGSpace(ScaleSpace& ss):
	noctave(ss.noctave), nscale(ss.nscale) {
	origw = ss.origw, origh = ss.origh;
	dogs = new shared_ptr<DOG>[noctave];

#pragma omp parallel for schedule(dynamic)
	REP(i, noctave)
		dogs[i] = shared_ptr<DOG>(new DOG(ss.octaves[i]));
}

DOGSpace::~DOGSpace()
{ delete[] dogs; }
