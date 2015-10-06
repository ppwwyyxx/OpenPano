//File: feature.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "feature.hh"

#include "extrema.hh"
#include "orientation.hh"
#include "sift.hh"
#include "dog.hh"
#include "lib/imgproc.hh"
using namespace std;

vector<Descriptor> detect_SIFT(const Mat32f& mat) {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	OrientationAssign ort(sp, ss, keyp);
	keyp = ort.work();
	SIFT sift(ss, keyp);
	auto ret = sift.get_descriptor();
	cout << "number of features: " << ret.size() << endl;
	return ret;
}


#ifdef __SSE3__
#include <x86intrin.h>

// %35 faster than base
float Descriptor::euclidean_sqr_fast(const Descriptor& r, float now_thres) const {
	float ans = 0;
	const float *x = descriptor.data(),
							*y = r.descriptor.data();
	__m128 vsum = _mm_set1_ps(0.0f);
	int n = descriptor.size();
	m_assert(n % 4 == 0);
	for (; n > 0; n -= 4) {
		const __m128 a = _mm_loadu_ps(x);
		const __m128 b = _mm_loadu_ps(y);
		const __m128 diff = _mm_sub_ps(a, b);
		const __m128 sqr = _mm_mul_ps(diff, diff);
		vsum = _mm_add_ps(vsum, sqr);

		// check result temporarily. slightly faster
		if (n % 32 == 0) {
			__m128 rst = _mm_hadd_ps(vsum, vsum);
			rst = _mm_hadd_ps(rst, rst);
			_mm_store_ss(&ans, rst);
			if (ans > now_thres)
				return -1;
		}

		x += 4;
		y += 4;
		_mm_prefetch(x, _MM_HINT_T0);
		_mm_prefetch(y, _MM_HINT_T0);
	}
	__m128 rst = _mm_hadd_ps(vsum, vsum);
	rst = _mm_hadd_ps(rst, rst);
	_mm_store_ss(&ans, rst);
	return ans;
}
#endif
