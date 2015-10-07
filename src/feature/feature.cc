//File: feature.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "feature.hh"

#include "extrema.hh"
#include "orientation.hh"
#include "sift.hh"
#include "dog.hh"
#include "brief.hh"
#include "lib/imgproc.hh"
using namespace std;


namespace feature {

vector<Descriptor> FeatureDetector::detect_feature(const Mat32f& img) const {
	auto ret = do_detect_feature(img);
	// convert scale-coordinate to half-offset image coordinate
	for (auto& d: ret) {
		d.coor.x = (d.coor.x - 0.5) * img.width();
		d.coor.y = (d.coor.y - 0.5) * img.height();
	}
	cout << "number of features: " << ret.size() << endl;
	return ret;
}

vector<Descriptor> SIFTDetector::do_detect_feature(const Mat32f& mat) const {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	OrientationAssign ort(sp, ss, keyp);
	keyp = ort.work();
	SIFT sift(ss, keyp);
	auto ret = sift.get_descriptor();
	return ret;
}

BRIEFDetector::BRIEFDetector() {
	pattern.reset(new BriefPattern(
				BRIEF::gen_brief_pattern(BRIEF_PATH_SIZE, BRIEF_NR_PAIR)));
}

BRIEFDetector::~BRIEFDetector() {}

vector<Descriptor> BRIEFDetector::do_detect_feature(const Mat32f& mat) const {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	//OrientationAssign ort(sp, ss, keyp);
	//keyp = ort.work();
	BRIEF brief(mat, keyp, *pattern);

	auto ret = brief.get_descriptor();
	return ret;
}

#ifdef __SSE3__
#include <x86intrin.h>

// %35 faster
float Descriptor::euclidean_sqr(const Descriptor& r, float now_thres) const {
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

#else

float Descriptor::euclidean_sqr(const Descriptor& r, float now_thres) const {
	float ans = 0;
	REP(i, descriptor.size()) {
		ans += sqr(descriptor[i] - r.descriptor[i]);
		if (ans > now_thres)
			return -1;
	}
	return ans;
}

#endif

int Descriptor::hamming(const Descriptor& r) const {
	int sum = 0;
	REP(i, descriptor.size()) {
		unsigned int* p1 = (unsigned int*)&descriptor[i];
		unsigned int* p2 = (unsigned int*)&r.descriptor[i];
		sum += __builtin_popcount((*p1) ^ *(p2));
	}
	return sum;
}
}
