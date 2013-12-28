// File: keypoint.cc
// Date: Sat Dec 28 20:15:52 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <cstring>
#include "keypoint.hh"
#include "matrix.hh"

using namespace std;

#define D(x, y, s) nowpic->get(s)->get_pixel(y, x)

KeyPoint::KeyPoint(const DOGSpace& m_dog, const ScaleSpace& m_ss):
	dogsp(m_dog),ss(m_ss)
{ noctave = dogsp.noctave, nscale = dogsp.nscale; }

void KeyPoint::detect_feature() {
	REP(i, noctave) REPL(j, 1, nscale - 2)
		judge_extrema(i, j);
}

void KeyPoint::judge_extrema(int nowo, int nows) {
	shared_ptr<GreyImg> now = dogsp.dogs[nowo]->get(nows);
	int w = now->w, h = now->h;
#pragma omp parallel for schedule(dynamic)
	REPL(i, 1, h - 1)
		REPL(j, 1, w - 1) {
			real_t nowcolor = now->get_pixel(i, j);
			if (nowcolor < PRE_COLOR_THRES)			// initial color is less than thres
				continue;
			if (judge_extrema(nowcolor, nowo, nows, i, j)) {
#pragma omp critical
				keyp.push_back(Coor((real_t)j / w * dogsp.origw, (real_t)i / h * dogsp.origh));

				get_feature(nowo, nows, i, j);		// i is h
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
	f.real_coor = Vec2D((real_t)newx / w * dogsp.origw, (real_t)newy / h * dogsp.origh);
	f.ns = news, f.no = nowo;
	f.scale_factor = GAUSS_SIGMA * pow(SCALE_FACTOR, (real_t)(offset.z + news) / nscale);
#pragma omp critical
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

	Matrix m(3, 3);
	m.get(0, 0) = dxx; m.get(1, 1) = dyy; m.get(2, 2) = dss;
	m.get(0, 1) = m.get(1, 0) = dxy;
	m.get(0, 2) = m.get(2, 0) = dsx;
	m.get(1, 2) = m.get(2, 1) = dys;

	Matrix inv(3, 3);			// formula 3
#define mul(l) inv.get(l, 0) * (*dx) + inv.get(l, 1) * (*dy) + inv.get(l, 2) * (*ds)
	if (m.inverse(inv))
		ret = Vec(-mul(0), -mul(1), -mul(2)); 		// which is better?
#undef mul
	return ret;
}

bool KeyPoint::judge_extrema(real_t center, int no, int ns, int nowi, int nowj) {		// i is h
	bool max = true, min = true;

	for (int level : {ns, ns - 1, ns + 1})
		REPL(di, -1, 2) REPL(dj, -1, 2) {
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
	update_feature.reserve(features.size());
	for (auto &feat : features)
		calc_dir(feat, update_feature);
	cout << "features: " << update_feature.size() << endl;
	features = move(update_feature);
}

// assign orientation to each keypoint
void KeyPoint::calc_dir(Feature& feat, vector<Feature>& update_feat) {
	int no = feat.no, ns = feat.ns;
	Coor now = feat.coor;

	for (auto ori : calc_hist(ss.octaves[no], ns, now, feat.scale_factor)) {
		Feature newf(feat);
		newf.dir = ori;
		update_feat.push_back(move(newf));
	}
}

vector<real_t> KeyPoint::calc_hist(shared_ptr<Octave> oct, int ns, Coor coor, real_t orig_sig) {		// coor is under scaled space
	real_t sigma = orig_sig * ORI_WINDOW_FACTOR;
	int rad = round(orig_sig * ORI_RADIUS);

	real_t exp_denom = 2 * sqr(sigma);
	real_t hist[ORT_HIST_BIN_NUM];
	memset(hist, 0, sizeof(hist));

	for (int xx = -rad; xx < rad; xx ++) {
		int newx = coor.x + xx;
		if (! between(newx, 1, oct->w - 1))		// because mag/ort on the border is zero
			continue;
		for (int yy = -rad; yy < rad; yy ++) {
			int newy = coor.y + yy;
			if (! between(newy, 1, oct->h - 1)) continue;
			if (sqr(xx) + sqr(yy) > sqr(rad)) continue;  // use a circular gaussian window
			int bin = round(ORT_HIST_BIN_NUM / 2 * (oct->get_ort(ns)->get_pixel(newy, newx)) / M_PI);
			if (bin == ORT_HIST_BIN_NUM) bin = 0;

			m_assert(bin < ORT_HIST_BIN_NUM);

			real_t weight = exp(-(sqr(xx) + sqr(yy)) / exp_denom);
			hist[bin] += weight * oct->get_mag(ns)->get_pixel(newy, newx);

			m_assert(hist[bin] >= 0);
		}
	}


	for (int K = ORT_HIST_SMOOTH_COUNT; K --;)		// smooth the histogram by interpolation
		REP(i, ORT_HIST_BIN_NUM) {
			real_t prev = hist[i == 0 ? ORT_HIST_BIN_NUM - 1 : i - 1];
			real_t next = hist[i == ORT_HIST_BIN_NUM - 1 ? 0 : i + 1];
			hist[i] = hist[i] * 0.5 + (prev + next) * 0.25;
		}

	real_t maxbin = 0;
	for (auto i : hist) update_max(maxbin, i);

	real_t thres = maxbin * ORT_HIST_PEAK_RATIO;
	vector<real_t> ret;

	REP(i, ORT_HIST_BIN_NUM) {
		real_t prev = hist[i == 0 ? ORT_HIST_BIN_NUM - 1 : i - 1];
		real_t next = hist[i == ORT_HIST_BIN_NUM - 1 ? 0 : i + 1];
		if (hist[i] > thres && hist[i] > max(prev, next)) {
			real_t newbin = i + (prev - next) / (prev + next - 2 * hist[i]) / 2;		// interpolation
			if (newbin < 0)
				newbin += ORT_HIST_BIN_NUM;
			else if (newbin >= ORT_HIST_BIN_NUM)
				newbin -= ORT_HIST_BIN_NUM;
			real_t ort = newbin / ORT_HIST_BIN_NUM * 2 * M_PI;
			ret.push_back(ort);
		}
	}
	m_assert(ret.size());
	return move(ret);
}

void KeyPoint::calc_descriptor() {
	int n = features.size();
#pragma omp parallel for schedule(dynamic)
	REP(i, n) calc_descriptor(features[i]);
}

void KeyPoint::calc_descriptor(Feature& feat) {
	static real_t pi2 = 2 * M_PI;
	static real_t nbin_per_rad = DESC_HIST_BIN_NUM / pi2;

	Coor coor = feat.coor;
	real_t ort = feat.dir,
		   hist_w = feat.scale_factor * DESC_HIST_REAL_WIDTH,
		   exp_denom = 2 * sqr(DESC_HIST_WIDTH / 2);		// from lowe

	int radius = round(sqrt(2) * hist_w * (DESC_HIST_WIDTH + 1) / 2);

	shared_ptr<Octave> octave = ss.octaves[feat.no];
	int w = octave->w, h = octave->h;

	real_t hist[DESC_HIST_WIDTH * DESC_HIST_WIDTH][DESC_HIST_BIN_NUM];
	memset(hist, 0, sizeof(hist));

	real_t cosort = cos(ort),
		   sinort = sin(ort);

	for (int xx = -radius; xx <= radius; xx ++) {
		int newx = coor.x + xx;
		if (!between(newx, 1, w - 1)) continue;
		for (int yy = -radius; yy <= radius; yy ++) {
			int newy = coor.y + yy;
			if (!between(newy, 1, h - 1)) continue;
			if (sqr(xx) + sqr(yy) > sqr(radius)) continue;		// to be circle

			real_t y_rot = (-xx * sinort + yy * cosort) / hist_w,		// rotate coor, let ort be x
				   x_rot = (xx * cosort + yy * sinort) / hist_w;
			real_t ybin_r = y_rot + DESC_HIST_WIDTH / 2 - 0.5,
				   xbin_r = x_rot + DESC_HIST_WIDTH / 2 - 0.5;

			real_t pic_mag = octave->get_mag(feat.ns)->get_pixel(newy, newx),
				   pic_ort = octave->get_ort(feat.ns)->get_pixel(newy, newx);

			if (between(ybin_r, -1, DESC_HIST_WIDTH) && between(xbin_r, -1, DESC_HIST_WIDTH)) {
				real_t win = exp(-(sqr(x_rot) + sqr(y_rot)) / exp_denom);

				pic_ort -= ort;							// for rotation invariance
				if (pic_ort < 0) pic_ort += pi2;
				if (pic_ort > pi2) pic_ort -= pi2;

				real_t nowbin_r = pic_ort * nbin_per_rad;
				int ybin = floor(ybin_r),
					xbin = floor(xbin_r),
					nowbin = floor(nowbin_r);

				// trilinear - to split a point to two block
				real_t ybin_d = ybin_r - ybin,
					   xbin_d = xbin_r - xbin,
					   nowbin_d = nowbin_r - nowbin;
				for (int r : {0, 1})
					if (between(ybin + r, 0, DESC_HIST_WIDTH)) {
						real_t v_y = pic_mag * (r ? ybin_d : 1 - ybin_d) * win;
						for (int c : {0, 1})
							if (between(xbin + c, 0, DESC_HIST_WIDTH)) {
								real_t v_x = v_y * (c ? xbin_d : 1 - xbin_d);
								for (int o : {0, 1}) {
									int ob = (nowbin + o) % DESC_HIST_BIN_NUM;
									real_t v_o = v_x * (o ? nowbin_d : 1 - nowbin_d);
									hist[(ybin + r) * DESC_HIST_WIDTH + (xbin + c)][ob] += v_o;
								}
							}
					} // end of trilinear
				/*
				 *if (between(ybin, 0, DESC_HIST_WIDTH) && between(xbin, 0, DESC_HIST_WIDTH))
				 *    hist[ybin * DESC_HIST_WIDTH + xbin][nowbin % DESC_HIST_BIN_NUM] += pic_mag * win;
				 */
			}
		}
	}
	memcpy(feat.descriptor, hist, sizeof(hist));
	// judge hist correctness
	m_assert(feat.descriptor[125] == hist[15][5]);

	// normalize and cut and normalize
	real_t sum = 0;
	for (auto &i : feat.descriptor) sum += sqr(i);
	sum = sqrt(sum);
	for (auto &i : feat.descriptor) {
		i /= sum;
		update_min(i, DESC_NORM_THRESH);
	}
	sum = 0;
	for (auto &i : feat.descriptor) sum += sqr(i);
	sum = sqrt(sum);
	for (auto &i : feat.descriptor) i = i / sum * DESC_INT_FACTOR;
}

#undef D
