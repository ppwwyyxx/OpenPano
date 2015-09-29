// File: filter.hh
// Date: Sat May 04 01:33:12 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <vector>
#include <cstring>
#include "lib/mat.h"
#include "lib/color.hh"

class GaussCache {
	public:
		float ** kernel;
		float kernel_tot;
		float normalization_factor;

		int kw;

		GaussCache(float sigma);

		~GaussCache()
		{ free_2d<float>(kernel, kw); }

		// something bad
		GaussCache(const GaussCache& m) {
			kernel_tot = m.kernel_tot;
			normalization_factor = m.normalization_factor;
			kw = m.kw;
			kernel = new float* [kw];
			REP(i, kw) {
				kernel[i] = new float[kw]();
				memcpy(kernel[i], m.kernel[i], kw * sizeof(float));
			}
		}

		GaussCache & operator = (const GaussCache & m) {
			if (this != &m) {
				kernel_tot = m.kernel_tot;
				normalization_factor = m.normalization_factor;
				kw = m.kw;
				kernel = new float* [kw];
				REP(i, kw) {
					kernel[i] = new float[kw]();
					memcpy(kernel[i], m.kernel[i], kw * sizeof(float));
				}
			}
			return *this;
		}

		GaussCache & operator = (GaussCache && r) {
			m_assert(this != &r);
			free_2d<float>(kernel, kw);
			kernel_tot = r.kernel_tot;
			normalization_factor = r.normalization_factor;
			kw = r.kw;
			kernel = r.kernel;
			r.kernel = nullptr;
			return *this;
		}

		GaussCache(GaussCache&& m) {
			kernel_tot = m.kernel_tot;
			normalization_factor = m.normalization_factor;
			kw = m.kw;
			kernel = m.kernel;
			m.kernel = nullptr;
		}

};

class Filter {
	public:
		std::vector<GaussCache> gcache;		// size = nscale - 1
		Filter(int nscale, float gauss_sigma, float scale_factor);

		Mat32f GaussianBlur(const Mat32f&, const GaussCache&) const;

		Mat32f GaussianBlur(const Mat32f& img, int n) const
		{ return GaussianBlur(img, gcache[n - 1]); }
};
