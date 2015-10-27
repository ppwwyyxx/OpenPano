//File: dist.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once


namespace feature {

float euclidean_sqr(
		const float* x, const float* y,
		int n, float now_thres);

int hamming(const float* x, const float* y, int n);

}
