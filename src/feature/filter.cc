// File: filter.cc
// Date: Thu Jul 04 11:05:14 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "lib/config.hh"
#include "filter.hh"
#include "lib/utils.hh"
#include "lib/timer.hh"
using namespace std;


namespace feature {

GaussCache::GaussCache(float sigma) {
	/*
	 *const int kw = round(GAUSS_WINDOW_FACTOR * sigma) + 1;
	 */
	kw = ceil(0.3 * (sigma / 2 - 1) + 0.8) * GAUSS_WINDOW_FACTOR;
	if (kw % 2 == 0) kw ++;
	// TODO decide window size ?

	const int center = kw / 2;
	normalization_factor = 2 * M_PI * sqr(sigma);
	kernel_tot = 0;

	kernel = new float[kw * kw];
	REP(i, kw) {
		REP(j, kw) {
			float x = i - center,
				    y = j - center;
			int loc = i * kw + j;
			kernel[loc] = exp(-(sqr(x) + sqr(y)) / (2 * sqr(sigma)));
			kernel[loc] /= normalization_factor;
			kernel_tot += kernel[loc];
		}
	}
}

Filter::Filter(int nscale,
		float gauss_sigma,
		float scale_factor) {
	REP(k, nscale - 1) {
		gcache.emplace_back(gauss_sigma);
		gauss_sigma *= scale_factor;
	}
}

// TODO fast convolution
Mat32f Filter::GaussianBlur(
		const Mat32f& img, const GaussCache& gauss) const {
	m_assert(img.channels() == 1);
	TotalTimer tm("gaussianblur");
	const int w = img.width(), h = img.height();
	Mat32f ret(h, w, 1);

	const int kw = gauss.kw;
	const int center = kw / 2;
	float * kernel = gauss.kernel;

	// cache. speed up a lot
	vector<const float*> row_ptrs(h);
	REP(i, h) row_ptrs[i] = img.ptr(i);

	REP(i, h) {
		float* rst_row = ret.ptr(i);
		REP(j, w) {
			int x_bound = min(kw, h + center - i),
					y_bound = min(kw, w + center - j);

			// perform a direct zero-padded convolution is good enough
			float kernel_tot = gauss.kernel_tot;
			// when deleted, cause large error in myself/small*. work later.
			if (not (j >= center && x_bound == kw && i >= center && y_bound == kw)) {
				kernel_tot = 0.f;
				for (int x = max(center - i, 0); x < x_bound; x ++)
					for (int y = max(center - j, 0); y < y_bound; y ++)
						kernel_tot += kernel[x * kw + y];
			}

			float newvalue = 0;
			for (int x = max(0, center - i); x < x_bound; x ++) {
				int di = x - center + i;
				const float* now_row_ptr = row_ptrs[di];
				const float* now_krn = kernel + x * kw;
				for (int y = max(0, center - j); y < y_bound; y ++) {
					int dj = y - center + j;
					float curr = now_row_ptr[dj];
					newvalue += curr * now_krn[y];
				}
			}
			float compensation = 1.0 / kernel_tot;
			rst_row[j] = newvalue * compensation;
		}
	}
	return ret;
}


}
