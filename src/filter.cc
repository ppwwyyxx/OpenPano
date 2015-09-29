// File: filter.cc
// Date: Thu Jul 04 11:05:14 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "lib/config.hh"
#include "filter.hh"
#include "lib/utils.hh"
#include "lib/timer.hh"
using namespace std;

GaussCache::GaussCache(float sigma) {
	/*
	 *const int kw = round(GAUSS_WINDOW_FACTOR * sigma) + 1;
	 */
	kw = ceil(0.3 * (sigma / 2 - 1) + 0.8) * GAUSS_WINDOW_FACTOR;
	// TODO decide window size ?

	const int center = kw / 2;
	normalization_factor = 2 * M_PI * sqr(sigma);
	kernel_tot = 0;

	kernel = new float*[kw];
	REP(i, kw) {
		kernel[i] = new float[kw];
		REP(j, kw) {
			float x = i - center,
				    y = j - center;
			kernel[i][j] = exp(-(sqr(x) + sqr(y)) / (2 * sqr(sigma)));
			kernel[i][j] /= normalization_factor;
			kernel_tot += kernel[i][j];
		}
	}
}

Filter::Filter(int nscale,
		float gauss_sigma,
		float scale_factor) {
	REP(k, nscale - 1) {
		gcache.emplace_back(GaussCache(gauss_sigma));
		gauss_sigma *= scale_factor;
	}
}

Mat32f Filter::GaussianBlur(
		const Mat32f& img, const GaussCache& gauss) const {
	m_assert(img.channels() == 1);
	TotalTimer tm("gaussianblur");
	const int w = img.width(), h = img.height();
	Mat32f ret(h, w, 1);

	const int kw = gauss.kw;
	const int center = kw / 2;
	float ** kernel = gauss.kernel;

	// cache. speed up a lot
	vector<const float*> row_ptrs(h);
	REP(i, h) row_ptrs[i] = img.ptr(i);

	REP(i, h) {
		float* now_row = ret.ptr(i);
		REP(j, w) {
			int x_bound = min(kw, h + center - i),
					y_bound = min(kw, w + center - j);

			float kernel_tot = 0;
			if (j >= center && x_bound == kw && i >= center && y_bound == kw)
				kernel_tot = gauss.kernel_tot;
			else {
				for (int x = max(center - i, 0); x < x_bound; x ++)
					for (int y = max(center - j, 0); y < y_bound; y ++)
						kernel_tot += kernel[x][y];
			}

			float compensation = 1.0 / kernel_tot;
			float newvalue = 0;
			for (int x = max(0, center - i); x < x_bound; x ++)
				for (int y = max(0, center - j); y < y_bound; y ++) {
					int dj = y - center + j,
							di = x - center + i;
					float curr = *(row_ptrs[di] + dj);
					newvalue += curr * kernel[x][y] * compensation;
				}
			now_row[j] = newvalue;
		}
	}
	return ret;
}

