// File: filter.cc
// Date: Thu Jul 04 11:05:14 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "config.hh"
#include "filter.hh"
#include "utils.hh"
#include "render/MImageRender.hh"
using namespace Magick;
using namespace std;

GaussCache::GaussCache(real_t sigma) {
	/*
	 *const int kw = round(GAUSS_WINDOW_FACTOR * sigma) + 1;
	 */
	kw = ceil(0.3 * (sigma / 2 - 1) + 0.8) * GAUSS_WINDOW_FACTOR;
	// TODO decide window size ?

	const int center = kw / 2;
	normalization_factor = 2 * M_PI * sqr(sigma);
	kernel_tot = 0;

	kernel = new real_t*[kw];
	REP(i, kw) {
		kernel[i] = new real_t[kw];
		REP(j, kw) {
			real_t x = i - center,
				   y = j - center;
			kernel[i][j] = exp(-(sqr(x) + sqr(y)) / (2 * sqr(sigma)));
			kernel[i][j] /= normalization_factor;
			kernel_tot += kernel[i][j];
		}
	}

}

Filter::Filter(int nscale, real_t gauss_sigma, real_t scale_factor) {
	REPL(k, 1, nscale) {
		gcache.push_back(GaussCache(gauss_sigma));
		gauss_sigma *= scale_factor;
	}
}

shared_ptr<GreyImg> Filter::GaussianBlur(const shared_ptr<GreyImg>& img,
										const GaussCache& gauss) const {
	const int w = img->w, h = img->h;
	shared_ptr<GreyImg> ret = make_shared<GreyImg>(w, h);

	const int kw = gauss.kw;
	const int center = kw / 2;
	real_t ** kernel = gauss.kernel;


	REP(i, h) REP(j, w) {
		int x_bound = min(kw, h + center - i),
			y_bound = min(kw, w + center - j);
		real_t kernel_tot = 0;
		if (j >= center && x_bound == kw && i >= center && y_bound == kw)
			kernel_tot = gauss.kernel_tot;
		else {
			for (int x = max(center - i, 0); x < x_bound; x ++)
				for (int y = max(center - j, 0); y < y_bound; y ++)
					kernel_tot += kernel[x][y];
		}

		real_t compensation = 1.0 / kernel_tot;
		real_t newvalue = 0;
		for (int x = max(0, center - i); x < x_bound; x ++)
			for (int y = max(0, center - j); y < y_bound; y ++) {
				int dj = y - center + j,
					di = x - center + i;
				real_t curr = img->get_pixel(di, dj);
				newvalue += curr * kernel[x][y] * compensation;
			}
		ret->set_pixel(i, j, newvalue);
	}
	return move(ret);
}

shared_ptr<GreyImg> Filter::GreyScale(const shared_ptr<Img> img) {
	shared_ptr<GreyImg> ret = make_shared<GreyImg>(*img);
	return move(ret);
}

real_t Filter::to_grey(const ::Color& c) {
	/*
	 *real_t ret = 0.299 * c.x + 0.587 * c.y + 0.114 * c.z;
	 */
	real_t ret = (c.x + c.y + c.z) / 3;
	return ret;
}
