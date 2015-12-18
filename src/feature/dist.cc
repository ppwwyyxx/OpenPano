//File: dist.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "dist.hh"
#include "lib/debugutils.hh"
#include "lib/utils.hh"
#include "lib/timer.hh"

#include <limits>

namespace pano {

#if defined(__SSE3__) || defined(__AVX__) || (_M_IX86_FP >= 2)
#ifdef _MSC_VER
#include <nmmintrin.h>
#else
#include <x86intrin.h>
#endif

// %35 faster
float euclidean_sqr(
		const float* x, const float* y,
		size_t n, float now_thres) {
	/*
	 *static long long cnt = 0;
	 *cnt ++; if (cnt % 10000000 == 0) PP(cnt);
	 */
	float ans = 0;
	__m128 vsum = _mm_set1_ps(0.0f);
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
				return std::numeric_limits<float>::max();
		}

		x += 4;
		y += 4;
		_mm_prefetch((char*)x, _MM_HINT_T0);
		_mm_prefetch((char*)y, _MM_HINT_T0);
	}
	__m128 rst = _mm_hadd_ps(vsum, vsum);
	rst = _mm_hadd_ps(rst, rst);
	_mm_store_ss(&ans, rst);
	return ans;
}

#else

float euclidean_sqr(
		const float* x, const float* y,
		size_t size, float now_thres) {
	m_assert(size % 4 == 0);
	float ans = 0;
	float diff0, diff1, diff2, diff3;
	const float* end = x + size;
	while (x < end) {
		diff0 = x[0] - y[0];
		diff1 = x[1] - y[1];
		diff2 = x[2] - y[2];
		diff3 = x[3] - y[3];
		ans += sqr(diff0) + sqr(diff1) + sqr(diff2) + sqr(diff3);
		if (ans > now_thres)
			return std::numeric_limits<float>::max();
		x += 4, y += 4;
	}
	return ans;
}

#endif

#ifdef _MSC_VER
#if defined(__AVX__) || (_M_IX86_FP >= 2)
#  include <nmmintrin.h>
#  define __builtin_popcount _mm_popcnt_u32
#else
#  include <intrin.h>
#  define __builtin_popcount __popcnt
#endif
#endif

int hamming(const float* x, const float* y, int n) {
	int sum = 0;
	REP(i, n) {
		unsigned int* p1 = (unsigned int*)&x[i];
		unsigned int* p2 = (unsigned int*)&y[i];
		sum += __builtin_popcount((*p1) ^ *(p2));
	}
	return sum;
}

}
