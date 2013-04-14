// File: sift.cc
// Date: Sun Apr 14 20:14:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <omp.h>
#include "sift.hh"
#include "filter.hh"
#include "matrix.hh"
using namespace std;

#define D(x, y, s) nowdog->get(s)->get_pixel(y, x)

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

Extrema::Extrema(const DOGSpace& m_dog):dogsp(m_dog)
{ noctave = dogsp.noctave, nscale = dogsp.nscale; }

void Extrema::detect_extrema() {
	for (int i = 0; i < noctave; i ++)
		for (int j = 1; j < nscale - 2; j ++)
			judge_extrema(i, j);
}

void Extrema::judge_extrema(int nowo, int nows) {
	shared_ptr<GreyImg> now = dogsp.dogs[nowo]->get(nows);
	int w = now->w, h = now->h;
	for (int i = 1; i < h - 1; i ++)
		for (int j = 1; j < w - 1; j ++) {
			real_t nowcolor = now->get_pixel(i, j);
			if (nowcolor < PRE_COLOR_THRES)			// initial color is less than thres
				continue;
			if (judge_extrema(nowcolor, nowo, nows, i, j)) {
				get_feature(nowo, nows, i, j);		// i is h
				/*
				 *keyp.push_back(Coor((real_t)j / w * dogsp.origw, (real_t)i / h * dogsp.origh));
				 */
				/*
				 *continue;
				 */
			}
		}
}

void Extrema::get_feature(int nowo, int nows, int r, int c) {
	shared_ptr<DOG> nowdog = dogsp.dogs[nowo];
	int w = nowdog->get(nows)->w,
		h = nowdog->get(nows)->h;
	int depth = 0;
	int newx = c, newy = r, news = nows;
	Vec offset;
	real_t dx, dy, ds;
	while (depth < SIFT_INTERPOLATION_DEPTH) {
#define between(a, b, c) ((a >= b) && (a < c))
		if (!between(newx, 1, w - 1) ||
				!between(newy, 1, h - 1) ||
				!between(news, 1, nscale - 2))	// nscale - 1?
			return;
#undef between

		offset = interpolation_offset(newx, newy, news, nowdog, &dx, &dy, &ds);
		if (offset.get_abs_max() < INTERP_OFFSET_THRES) // found
			break;

		newx += round(offset.x);
		newy += round(offset.y);
		news += round(offset.z);

		depth ++;
	}

	if (depth == SIFT_INTERPOLATION_DEPTH) return;

	real_t dextr = offset.dot(Vec(dx, dy, ds));		// calc D(x~)
	dextr = D(newx, newy, news) + dextr / 2;
	if (dextr < CONTRAST_THRES)
		return;			// contrast too low

	keyp.push_back(Coor((real_t)newx / w * dogsp.origw, (real_t)newy / h * dogsp.origh));
}

Vec Extrema::interpolation_offset(int x, int y, int nows, shared_ptr<DOG>& nowdog,
		real_t* dx, real_t* dy, real_t* ds)
{
	Vec ret = Vector::get_zero();
	real_t dxx, dyy, dss, dxy, dys, dsx;
	// matrix
	real_t val = D(x, y, nows);

	*dx = (D(x + 1, y, nows) - D(x - 1, y, nows)) / 2;
	*dy = (D(x, y + 1, nows) - D(x, y - 1, nows)) / 2;
	*ds = (D(x, y, nows + 1) - D(x, y, nows - 1)) / 2;

	dxx = D(x + 1, y, nows) + D(x - 1, y, nows) - val - val;
	dyy = D(x, y + 1, nows) + D(x, y - 1, nows) - val - val;
	dss = D(x, y, nows + 1) + D(x, y, nows - 1) - val - val;

	dxy = (D(x + 1, y + 1, nows) - D(x + 1, y - 1, nows) - D(x - 1, y + 1, nows) + D(x - 1, y - 1, nows)) / 4;
	dys = (D(x, y + 1, nows + 1) - D(x, y - 1, nows + 1) - D(x, y + 1, nows - 1) + D(x, y - 1, nows - 1)) / 4;
	dsx = (D(x + 1, y, nows + 1) - D(x - 1, y, nows + 1) - D(x + 1, y, nows - 1) + D(x - 1, y, nows - 1)) / 4;

	Mat m(3, 3);
	m.get(0, 0) = dxx; m.get(1, 1) = dyy; m.get(2, 2) = dss;
	m.get(0, 1) = m.get(1, 0) = dxy;
	m.get(0, 2) = m.get(2, 0) = dsx;
	m.get(1, 2) = m.get(2, 1) = dys;

	Mat inv(3, 3);			// formula 3
#define mul(l) inv.get(l, 0) * (*dx) + inv.get(l, 1) * (*dy) + inv.get(l, 2) * (*ds)
	if (inverse(m, inv))
		ret = Vec(-mul(0), -mul(1), -mul(2)); 		// seem better?
#undef mul
	return move(ret);
}

bool Extrema::judge_extrema(real_t center, int no, int ns, int nowi, int nowj) {
	bool max = true, min = true;
#define judge(level)\
	do {\
		for (int di = -1; di <= 1; di ++)\
			for (int dj = -1; dj <= 1; dj ++) {\
				if (!di && !dj && level == ns) continue;\
				real_t newval = dogsp.dogs[no]->get(level)->get_pixel(nowi + di, nowj + dj);\
				if (newval >= center - JUDGE_EXTREMA_DIFF_THRES) max = false;\
				if (newval <= center + JUDGE_EXTREMA_DIFF_THRES) min = false;\
				if (!max && !min)\
					return false;\
			}\
	} while (0)

	judge(ns);		// ns first
	judge(ns - 1); judge(ns + 1);
#undef judge
	return true;

}
#undef D
