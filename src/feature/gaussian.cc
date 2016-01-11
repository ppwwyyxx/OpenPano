// File: gaussian.cc
// Date: Thu Jul 04 11:05:14 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <algorithm>
#include <cmath>
#include "gaussian.hh"
#include "lib/config.hh"
#include "lib/utils.hh"
#include "lib/timer.hh"
using namespace std;
using namespace config;


namespace pano {

GaussCache::GaussCache(float sigma) {
	// TODO decide window size ?
	/*
	 *const int kw = round(GAUSS_WINDOW_FACTOR * sigma) + 1;
	 */
	kw = ceil(0.3 * (sigma / 2 - 1) + 0.8) * GAUSS_WINDOW_FACTOR;
	//cout << kw << " " << sigma << endl;
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

// TODO faster convolution
template <typename T>
Mat<T> GaussianBlur::blur(const Mat<T>& img) const {
	//m_assert(img.channels() == 1);
	TotalTimer tm("gaussianblur");
	const int w = img.width(), h = img.height();
	Mat<T> ret(h, w, img.channels());

	const int kw = gcache.kw;
	const int center = kw / 2;
	float * kernel = gcache.kernel;

	vector<T> cur_line_mem(center * 2 + std::max(w, h), 0);
	T *cur_line = cur_line_mem.data() + center;

	// apply to columns
	REP(j, w){
		const T* src = img.ptr(0, j);
		// copy a column of src
		REP(i, h) {
			cur_line[i] = *src;
			src += w;
		}

		// pad the border with border value
		T v0 = cur_line[0];
		for (int i = 1; i <= center; i ++)
			cur_line[-i] = v0;
		v0 = cur_line[h - 1];
		for (int i = 0; i < center; i ++)
			cur_line[h + i] = v0;

		T *dest = ret.ptr(0, j);
		REP(i, h) {
			T tmp = 0;
			for (int k = -center; k <= center; k ++)
				tmp += cur_line[i + k] * kernel[k];
			*dest = tmp;
			dest += w;
		}
	}

	// apply to rows
	REP(i, h) {
		T *dest = ret.ptr(i);
		memcpy(cur_line, dest, sizeof(T) * w);
		{	// pad the border
			T v0 = cur_line[0];
			for (int j = 1; j <= center; j ++)
				cur_line[-j] = v0;
			v0 = cur_line[w - 1];
			for (int j = 0; j < center; j ++)
				cur_line[center + j] = v0;
		}
		REP(j, w) {
			T tmp = 0;
			for (int k = -center; k <= center; k ++)
				tmp += cur_line[j + k] * kernel[k];
			*(dest ++) = tmp;
		}
	}
	return ret;
}

template Mat32f GaussianBlur::blur<float>(const Mat32f&) const;

}
