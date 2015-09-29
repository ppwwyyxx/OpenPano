// File: sift.cc
// Date: Thu Jul 04 11:51:48 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "lib/config.hh"
#include "sift.hh"
#include "lib/utils.hh"
#include "lib/imgproc.hh"
#include "filter.hh"
using namespace std;

Octave::Octave(const Mat32f& m, int num_scale):
	nscale(num_scale),
	data(num_scale), mag(num_scale), ort(num_scale),
   w(m.width()), h(m.height())
{
	if (m.channels() == 3)
		data[0] = rgb2grey(m);
	else
		data[0] = m.clone();

	Filter blurer(nscale, GAUSS_SIGMA, SCALE_FACTOR);
	for (int i = 1; i < nscale; i ++) {
		data[i] = blurer.GaussianBlur(data[0], i);	// sigma needs a better one
		cal_mag_ort(i);
	}
}

void Octave::cal_mag_ort(int i) {
	const Mat32f& orig = data[i];
	int w = orig.width(), h = orig.height();
	mag[i] = Mat32f(h, w, 1);
	ort[i] = Mat32f(h, w, 1);
	REP(x, w) REP(y, h) {
		if (between(x, 1, w - 1) && between(y, 1, h - 1)) {
			real_t dy = orig.at(y + 1, x) - orig.at(y - 1, x),
				   dx = orig.at(y, x + 1) - orig.at(y, x - 1);
			mag[i].at(y, x) = hypot(dx, dy);
			if (dx == 0 && dy == 0)
				ort[i].at(y, x) = M_PI;
			else
				ort[i].at(y, x) = atan2(dy, dx) + M_PI;
		} else {
			mag[i].at(y, x) = 0;
			ort[i].at(y, x) = M_PI;
		}
	}
}

ScaleSpace::ScaleSpace(const Mat32f& mat, int num_octave, int num_scale):
	noctave(num_octave), nscale(num_scale){
		auto img = make_shared<Img>(mat);
		origw = mat.width(), origh = mat.height();
		octaves = new shared_ptr<Octave>[noctave];

		HWTimer timer;
// #pragma omp parallel for schedule(dynamic)
		REP(i, noctave) {
			if (!i)
				octaves[i] = make_shared<Octave>(img->mat, nscale);
			else {
				imgptr resized = make_shared<Img>(img->get_resized(pow(SCALE_FACTOR, -i)));
				octaves[i] = make_shared<Octave>(resized->mat, nscale);
			}
		}
#pragma omp critical
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

shared_ptr<GreyImg> DOG::diff(const Mat32f& img1, const Mat32f& img2) {
	int w = img1.width(), h = img1.height();
	m_assert(w == img2.width() && h == img2.height());
	shared_ptr<GreyImg> ret = make_shared<GreyImg>(w, h);
	REP(i, h) REP(j, w) {
		real_t diff = fabs(img1.at(i, j) - img2.at(i, j));
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
		dogs[i] = make_shared<DOG>(ss.octaves[i]);
}

DOGSpace::~DOGSpace()
{ delete[] dogs; }
