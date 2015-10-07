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
	// TODO decide window size ?
	/*
	 *const int kw = round(GAUSS_WINDOW_FACTOR * sigma) + 1;
	 */
	kw = ceil(0.3 * (sigma / 2 - 1) + 0.8) * GAUSS_WINDOW_FACTOR;
	if (kw % 2 == 0) kw ++;
	kernel_buf = create_auto_buf<float>(kw);
	const int center = kw / 2;
	kernel = kernel_buf.get() + center;

	kernel[0] = 1;

	float exp_coeff = -1.0 / (sigma * sigma * 2),
				 wsum = 1;
	for (int i = 1; i <= center; i ++)
		wsum += (kernel[i] = exp(i * i * exp_coeff)) * 2;

	float fac = 1.0 / wsum;
	kernel[0] = fac;
	for (int i = 1; i <= center; i ++)
		kernel[-i] = (kernel[i] *= fac);
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

	auto cur_line_mem = create_auto_buf<float>(
			center * 2 + max(w, h), true);
	float *cur_line = cur_line_mem.get() + center;

	// apply to columns
	REP(j, w){
		const float* src = img.ptr(0, j);
		// copy a column of src
		REP(i, h) {
			cur_line[i] = *src;
			src += w;
		}

		// pad the border with border value
		float v0 = cur_line[0];
		for (int i = 1; i <= center; i ++)
			cur_line[-i] = v0;
		v0 = cur_line[h - 1];
		for (int i = 0; i < center; i ++)
			cur_line[h + i] = v0;

		float *dest = ret.ptr(0, j);
		REP(i, h) {
			float tmp = 0;
			for (int k = -center; k <= center; k ++)
				tmp += kernel[k] * cur_line[i + k];
			*dest = tmp;
			dest += w;
		}
	}

	// apply to rows
	REP(i, h) {
		float *dest = ret.ptr(i);
		memcpy(cur_line, dest, sizeof(cur_line[0]) * w);
		{	// pad the border
			float v0 = cur_line[0];
			for (int j = 1; j <= center; j ++)
				cur_line[-j] = v0;
			v0 = cur_line[w - 1];
			for (int j = 0; j < center; j ++)
				cur_line[center + j] = v0;
		}
		REP(j, w) {
			float tmp = 0;
			for (int k = -center; k <= center; k ++)
				tmp += kernel[k] * cur_line[j + k];
			*(dest ++) = tmp;
		}
	}
	return ret;
}


}
