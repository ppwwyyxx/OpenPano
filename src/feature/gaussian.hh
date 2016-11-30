// File: gaussian.hh
// Date: Sat May 04 01:33:12 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <vector>
#include "lib/mat.h"
#include "lib/utils.hh"
#include "lib/timer.hh"
#include "common/common.hh"
namespace pano {

class GaussCache {
	public:
		std::unique_ptr<float, std::default_delete<float[]>> kernel_buf;
		float* kernel;
		int kw;
		GaussCache(float sigma);
};

class GaussianBlur {
	float sigma;
	GaussCache gcache;
	public:
		GaussianBlur(float sigma): sigma(sigma), gcache(sigma) {}

		// TODO faster convolution
		template <typename T>
		Mat<T> blur(const Mat<T>& img) const {
			m_assert(img.channels() == 1);
			TotalTimer tm("gaussianblur");
			const int w = img.width(), h = img.height();
			Mat<T> ret(h, w, img.channels());

			const int kw = gcache.kw;
			const int center = kw / 2;
			float * kernel = gcache.kernel;

			std::vector<T> cur_line_mem(center * 2 + std::max(w, h), 0);
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
					T tmp{0};
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
					T tmp{0};
					for (int k = -center; k <= center; k ++)
						tmp += cur_line[j + k] * kernel[k];
					*(dest ++) = tmp;
				}
			}
			return ret;
		}
};

class MultiScaleGaussianBlur {
	std::vector<GaussianBlur> gauss;		// size = nscale - 1
	public:
	MultiScaleGaussianBlur(
			int nscale, float gauss_sigma,
			float scale_factor) {
		REP(k, nscale - 1) {
			gauss.emplace_back(gauss_sigma);
			gauss_sigma *= scale_factor;
		}
	}

	Mat32f blur(const Mat32f& img, int n) const
	{ return gauss[n - 1].blur(img); }
};

}
