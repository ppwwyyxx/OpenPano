// File: filter.hh
// Date: Sat May 04 01:33:12 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <vector>
#include "lib/mat.h"

namespace feature {

class GaussCache {
	public:
		std::shared_ptr<float> kernel_buf;
		float* kernel;
		int kw;
		GaussCache(float sigma);
};

class Filter {
	public:
		std::vector<GaussCache> gcache;		// size = nscale - 1
		Filter(int nscale, float gauss_sigma, float scale_factor);

		Mat32f GaussianBlur(const Mat32f&, const GaussCache&) const;

		Mat32f GaussianBlur(const Mat32f& img, int n) const
		{ return GaussianBlur(img, gcache[n - 1]); }
};

}
