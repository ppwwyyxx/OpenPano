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
	kernel_buf.reset(new float[kw]);
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


}
