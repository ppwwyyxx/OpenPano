// File: gaussian.hh
// Date: Sat May 04 01:33:12 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <vector>
#include "lib/mat.h"
#include "lib/utils.hh"

namespace pano {

class GaussCache {
	public:
		std::shared_ptr<float> kernel_buf;
		float* kernel;
		int kw;
		GaussCache(float sigma);
};

class GaussianBlur {
	float sigma;
	GaussCache gcache;
	public:
		GaussianBlur(float sigma): sigma(sigma), gcache(sigma) {}
		Mat32f blur(const Mat32f&) const;
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
