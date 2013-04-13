// File: sift.cc
// Date: Sun Apr 14 01:19:33 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "sift.hh"
#include "filter.hh"
#include "matrix.hh"
using namespace std;

Octave::Octave(const shared_ptr<GreyImg>& img, int num_scale):
	nscale(num_scale){
	w = img->w, h = img->h;
	data = new std::shared_ptr<GreyImg>[nscale];
	data[0] = img;
	real_t s = pow(SCALE_FACTOR, nscale - 1);
	for (int i = 1; i < nscale; i ++)
		data[i] = Filter::GaussianBlur(data[i-1], s);
}

Octave::~Octave()
{ delete[] data; }

ScaleSpace::ScaleSpace(const shared_ptr<Img>& img, int num_octave, int num_scale):
	noctave(num_octave), nscale(num_scale){
	octaves = new shared_ptr<Octave>[noctave];
	octaves[0] = shared_ptr<Octave>(new Octave(img, nscale));
	for (int i = 1; i < noctave; i ++) {
		Img now = img->get_resized(pow(SCALE_FACTOR, -i));
		octaves[i] = shared_ptr<Octave>(new Octave(make_shared<Img>(now), nscale));
	}
}

ScaleSpace::~ScaleSpace()
{ delete[] octaves; }

DOG::DOG(const std::shared_ptr<Octave>& o) {
	nscale = o->get_len();
	data = new std::shared_ptr<GreyImg>[nscale - 1];
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
			/*
			 *if (diff < 16)
			 *    diff *= diff;
			 *else
			 *    diff = 255;
			 *if (diff < 10)
			 *    diff = 0;
			 */
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
	for (int i = 0; i < noctave; i ++)
		dogs[i] = shared_ptr<DOG>(new DOG(ss.octaves[i]));
}

DOGSpace::~DOGSpace()
{ delete[] dogs; }

Extrema::Extrema(const DOGSpace& m_dog):dogsp(m_dog){
	noctave = dogsp.noctave, nscale = dogsp.nscale;
}

void Extrema::detect_extrema() {
	print_debug("entering debug\n");
	for (int i = 0; i < noctave; i ++)
		for (int j = 1; j < nscale - 2; j ++) {
			shared_ptr<GreyImg> now = dogsp.dogs[i]->get(j);
			judge_extrema(now, i, j);
		}
}

void Extrema::judge_extrema(shared_ptr<GreyImg> now, int nowo, int nows) {
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

void Extrema::get_feature(int nowo, int nows, int i, int j) {	// i is h
	int w = dogsp.dogs[nowo]->get(nows)->w,
		h = dogsp.dogs[nowo]->get(nows)->h;
	int depth = 0, times;
	int newx = j, newy = i, news = nows;
	real_t xi, yi, si;
	real_t dx, dy, ds;
	while (depth < SIFT_INTERPOLATION_DEPTH) {
		interpolation_offset(newx, newy, news, nowo, &xi, &yi, &si, &dx, &dy, &ds);
		if (fabs(xi) < 0.5 && fabs(yi) < 0.5 && fabs(si) < 0.5)		// found
			break;

		newx += round(xi);
		newy += round(yi);
		news += round(si);
#define between(a, b, c) ((a >= b) && (a < c))
		if (!between(newx, 0, w) ||
				!between(newy, 0, h) ||
				!between(news, 1, nscale - 2))	// nscale - 1?
			return;
#undef between

		depth ++;
	}

	if (depth == SIFT_INTERPOLATION_DEPTH) return;

	real_t dextr = dx * xi + dy * yi + ds * si;
	dextr = dogsp.dogs[nowo]->get(news)->get_pixel(newx, newy) + dextr / 2;
	if (dextr < CONTRAST_THRES)
		return;			// contrast too low

	keyp.push_back(Coor((real_t)newx / w * dogsp.origw, (real_t)newy / h * dogsp.origh));
}

void Extrema::interpolation_offset(int x, int y, int nows, int nowo,
		real_t* xi, real_t* yi, real_t* si,
		real_t* pdx, real_t* pdy, real_t* pds)
{
	real_t dx, dy, ds;
	real_t dxx, dyy, dss, dxy, dys, dsx;
	// matrix
	shared_ptr<DOG> nowdog = dogsp.dogs[nowo];
	real_t val = nowdog->get(nows)->get_pixel(y, x);

#define D(x, y, s) nowdog->get(s)->get_pixel(x, y)

	dx =(D(x + 1, y, nows) - D(x - 1, y, nows)) / 2;
	dy =(D(x, y + 1, nows) - D(x, y - 1, nows)) / 2;
	ds =(D(x, y, nows + 1) - D(x, y, nows - 1)) / 2;

	dxx = D(x + 1, y, nows) + D(x - 1, y, nows) - val - val;
	dyy = D(x, y + 1, nows) + D(x, y - 1, nows) - val - val;
	dss = D(x, y, nows + 1) + D(x, y, nows - 1) - val - val;

	dxy = ((D(x + 1, y + 1, nows) - D(x + 1, y - 1, nows)) - (D(x - 1, y + 1, nows) - D(x - 1, y - 1, nows))) / 4;
	dys = ((D(x, y + 1, nows + 1) - D(x, y - 1, nows + 1)) - (D(x, y + 1, nows - 1) - D(x, y - 1, nows - 1))) / 4;
	dsx = ((D(x + 1, y, nows + 1) - D(x - 1, y, nows + 1)) - (D(x + 1, y, nows - 1) - D(x - 1, y, nows - 1))) / 4;
#undef D

	Mat m(3, 3);
	m.get(0, 0) = dxx; m.get(1, 1) = dyy; m.get(2, 2) = dss;
	m.get(0, 1) = m.get(1, 0) = dxy;
	m.get(0, 2) = m.get(2, 0) = dsx;
	m.get(1, 2) = m.get(2, 1) = dys;
	Mat inv(3, 3);
#define mul(l) inv.get(l, 0) * dx + inv.get(l, 1) * dy + inv.get(l, 2) * ds
	if (inverse(m, inv)) {
		*xi = mul(0);
		*yi = mul(1);
		*si = mul(2);
#undef mul
	} else {
		*xi = *yi = *si = 0;
	}
	cout << m << endl << inv << endl;
	*pdx = dx, *pdy = dy, *pds = ds;
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
	cout << no << ns << " " << nowi << " " << nowj << endl;
	return true;

}
