// File: keypoint.cc
// Date: Tue Apr 16 13:31:05 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <cstring>
#include "keypoint.hh"
#include "matrix.hh"

using namespace std;

#define D(x, y, s) nowpic->get(s)->get_pixel(y, x)
#define between(a, b, c) ((a >= b) && (a < c))

KeyPoint::KeyPoint(const DOGSpace& m_dog, const ScaleSpace& m_ss):
	dogsp(m_dog),ss(m_ss)
{ noctave = dogsp.noctave, nscale = dogsp.nscale; }

void KeyPoint::detect_extrema() {
	for (int i = 0; i < noctave; i ++)
		for (int j = 1; j < nscale - 2; j ++)
			judge_extrema(i, j);
}

void KeyPoint::judge_extrema(int nowo, int nows) {
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
				// to get original keypoints
			}
		}
}

void KeyPoint::get_feature(int nowo, int nows, int r, int c) {
	shared_ptr<DOG> nowpic = dogsp.dogs[nowo];
	int w = nowpic->get(nows)->w,
		h = nowpic->get(nows)->h;
	int depth = 0;
	int newx = c, newy = r, news = nows;
	Vec offset,		// x~
		delta;		// partial(d) / partial(x)
	real_t dx, dy, ds;
	while (depth < CALC_OFFSET_DEPTH) {
		if (!between(newx, 1, w - 1) ||
				!between(newy, 1, h - 1) ||
				!between(news, 1, nscale - 2))	// nscale - 1?
			return;

		offset = calc_offset(newx, newy, news, nowpic, &dx, &dy, &ds);
		if (offset.get_abs_max() < OFFSET_THRES) // found
			break;

		newx += round(offset.x);
		newy += round(offset.y);
		news += round(offset.z);

		depth ++;
	}

	if (depth == CALC_OFFSET_DEPTH) return;

	real_t dextr = offset.dot(Vec(dx, dy, ds));		// calc D(x~)
	dextr = D(newx, newy, news) + dextr / 2;
	if (dextr < CONTRAST_THRES) return;			// contrast too low

	if (on_edge(newx, newy, nowpic->get(news))) return;

	Feature f;
	f.coor = Coor(newx, newy);
	f.real_coor = Coor((real_t)newx / w * dogsp.origw,
			(real_t)newy / h * dogsp.origh);
	f.ns = news, f.no = nowo;
	f.sig_octave = GAUSS_SIGMA * pow(SCALE_FACTOR,
			(real_t)(offset.z + news) / nscale);
	f.sig_space = f.sig_octave * pow(SCALE_FACTOR, nowo);

	features.push_back(f);
}

bool KeyPoint::on_edge(int x, int y, const shared_ptr<GreyImg>& img) {
	real_t dxx, dxy, dyy;
	real_t val = img->get_pixel(y, x);

	dxx = img->get_pixel(y, x + 1) + img->get_pixel(y, x - 1) - val - val;
	dyy = img->get_pixel(y + 1, x) + img->get_pixel(y - 1, x) - val - val;
	dxy = (img->get_pixel(y + 1, x + 1) + img->get_pixel(y - 1, x - 1) -
			img->get_pixel(y + 1, x - 1) - img->get_pixel(y - 1, x + 1)) / 4;

	real_t det = dxx * dyy - dxy * dxy;
	if (det <= 0) return true;
	real_t tr = sqr(dxx + dyy);
	if (tr / det < sqr(EDGE_RATIO + 1) / EDGE_RATIO) return false;
	return true;
}

Vec KeyPoint::calc_offset(int x, int y, int nows, shared_ptr<DOG>& nowpic,
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
		ret = Vec(+mul(0), +mul(1), +mul(2)); 		// which is better?
#undef mul
	return ret;
}

bool KeyPoint::judge_extrema(real_t center, int no, int ns, int nowi, int nowj) {		// i is h
	bool max = true, min = true;

	for (int level : {ns, ns - 1, ns + 1})
		for (int di = -1; di <= 1; di ++)
			for (int dj = -1; dj <= 1; dj ++) {
				if (!di && !dj && level == ns) continue;
				real_t newval = dogsp.dogs[no]->get(level)->get_pixel(nowi + di, nowj + dj);
				if (newval >= center - JUDGE_EXTREMA_DIFF_THRES) max = false;
				if (newval <= center + JUDGE_EXTREMA_DIFF_THRES) min = false;
				if (!max && !min)
					return false;
			}
	return true;

}

void KeyPoint::calc_dir() {
	m_assert(features.size());  // require get_feature finished
	vector<Feature> update_feature;
	for (auto &feat : features)
		calc_dir(feat, update_feature);
	cout << "before assign: " << features.size() << endl;
	cout << "after assign: " << update_feature.size() << endl;
	features = move(update_feature);
}

void KeyPoint::calc_dir(Feature& feat, vector<Feature>& update_feat) {
	int no = feat.no, ns = feat.ns;
	Coor now = feat.coor;

	for (auto ori : calc_hist(ss.octaves[no], ns, now, feat.sig_octave)) {
		Feature newf(feat);
		newf.dir = ori;
		update_feat.push_back(move(newf));
	}
}

vector<real_t> KeyPoint::calc_hist(shared_ptr<Octave> oct, int ns, Coor coor, real_t orig_sig) {		// coor is under scaled space
	real_t sigma = orig_sig * ORI_WINDOW_FACTOR;
	int rad = orig_sig * ORI_RADIUS + 0.5;

	real_t exp_denom = 2 * sqr(sigma);
	real_t hist[HIST_BIN_NUM];
	memset(hist, 0, sizeof(hist));

	for (int xx = -rad; xx < rad; xx ++) {
		int newx = coor.x + xx;
		if (! between(newx, 1, oct->w - 1))		// because mag/ort on the border is zero
			continue;
		for (int yy = -rad; yy < rad; yy ++) {
			int newy = coor.y + yy;
			if (! between(newy, 1, oct->h - 1))
				continue;
			if (sqr(xx) + sqr(yy) > sqr(rad))		// use a circular gaussian window
				continue;
			int bin = round(HIST_BIN_NUM / 2 * (oct->get_ort(ns)->get_pixel(newy, newx)) / M_PI);
			if (bin == HIST_BIN_NUM) bin = 0;

			m_assert(bin < HIST_BIN_NUM);

			real_t weight = exp(-(sqr(xx) + sqr(yy)) / exp_denom);
			hist[bin] += weight * oct->get_mag(ns)->get_pixel(newy, newx);

			m_assert(hist[bin] >= 0);
		}
	}


	for (int kk = HIST_SMOOTH_COUNT; kk --;)		// smooth by interpolation
		for (int i = 0; i < HIST_BIN_NUM; i ++) {
			real_t prev = hist[i == 0 ? HIST_BIN_NUM - 1 : i - 1];
			real_t next = hist[i == HIST_BIN_NUM - 1 ? 0 : i + 1];
			hist[i] = hist[i] * 0.5 + (prev + next) * 0.25;
		}

	real_t maxbin = 0;
	for (auto i : hist) update_max(maxbin, i);

	real_t thres = maxbin * HIST_PEAK_RATIO;
	vector<real_t> ret;

	for (int i = 0; i < HIST_BIN_NUM; i ++) {
		real_t prev = hist[i == 0 ? HIST_BIN_NUM - 1 : i - 1];
		real_t next = hist[i == HIST_BIN_NUM - 1 ? 0 : i + 1];
		if (hist[i] > thres && hist[i] > max(prev, next)) {
			real_t newbin = i + (prev - next) / (prev + next - 2 * hist[i]) / 2;		// interpolation
			if (newbin < 0)
				newbin += HIST_BIN_NUM;
			else if (newbin >= HIST_BIN_NUM)
				newbin -= HIST_BIN_NUM;
			real_t ort = newbin / HIST_BIN_NUM * 2 * M_PI;
			ret.push_back(ort);
		}
	}
	m_assert(ret.size());
	return move(ret);
}

#undef D
#undef between
