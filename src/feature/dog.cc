// File: dog.cc
// Date: Thu Jul 04 11:51:48 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>

#include "lib/config.hh"
#include "dog.hh"
#include "lib/imgproc.hh"
#include "lib/timer.hh"
#include "lib/utils.hh"
#include "gaussian.hh"
using namespace std;
using namespace config;

namespace {
// fast approximation to atan2.
// atan2(a, b) = fast_atan(a, b), given max(abs(a),abs(b)) > EPS
// http://math.stackexchange.com/questions/1098487/atan2-faster-approximation
// save cal_mag_ort() 40% time
float fast_atan(float y, float x) {
	float absx = fabs(x), absy = fabs(y);
	float m = max(absx, absy);

	// undefined behavior in atan2.
	// but here we can safely ignore by setting ort=0
	if (m < EPS) return -M_PI;
	float a = min(absx, absy) / m;
	float s = a * a;
	float r = ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a;
	if (absy > absx)
		r = M_PI_2 - r;
	if (x < 0) r = M_PI - r;
	if (y < 0) r = -r;
	return r;
}
}

namespace pano {

GaussianPyramid::GaussianPyramid(const Mat32f& m, int num_scale):
	nscale(num_scale),
	data(num_scale), mag(num_scale), ort(num_scale),
	w(m.width()), h(m.height())
{
	TotalTimer tm("build pyramid");
	if (m.channels() == 3)
		data[0] = rgb2grey(m);
	else
		data[0] = m.clone();

	MultiScaleGaussianBlur blurer(nscale, GAUSS_SIGMA, SCALE_FACTOR);
	for (int i = 1; i < nscale; i ++) {
		data[i] = blurer.blur(data[0], i);	// sigma needs a better one
		cal_mag_ort(i);
	}
}

void GaussianPyramid::cal_mag_ort(int i) {
	TotalTimer tm("cal_mag_ort");
	const Mat32f& orig = data[i];
	int w = orig.width(), h = orig.height();
	mag[i] = Mat32f(h, w, 1);
	ort[i] = Mat32f(h, w, 1);
	REP(y, h) {
		float *mag_row = mag[i].ptr(y),
					*ort_row = ort[i].ptr(y);
		const float *orig_row = orig.ptr(y),
					*orig_plus = orig.ptr(y + 1),
					*orig_minus = orig.ptr(y - 1);
		// x == 0:
		mag_row[0] = 0;
		ort_row[0] = M_PI;

		REPL(x, 1, w-1) {
			if (between(y, 1, h - 1)) {
				float dy = orig_plus[x] - orig_minus[x],
							dx = orig_row[x + 1] - orig_row[x - 1];
				mag_row[x] = hypotf(dx, dy);
				// approx here cause break working on myself/small*. fix later
				// when dx==dy==0, no need to set ort
				ort_row[x] = fast_atan(dy, dx) + M_PI;
			} else {
				mag_row[x] = 0;
				ort_row[x] = M_PI;
			}
		}
		// x == w-1
		mag_row[w-1] = 0;
		ort_row[w-1] = M_PI;

	}
}

ScaleSpace::ScaleSpace(const Mat32f& mat, int num_octave, int num_scale):
	noctave(num_octave), nscale(num_scale),
	origw(mat.width()), origh(mat.height())
{
	// #pragma omp parallel for schedule(dynamic)
	REP(i, noctave) {
		if (!i)
			pyramids.emplace_back(mat, nscale);
		else {
			float factor = pow(SCALE_FACTOR, -i);
			int neww = ceil(origw * factor),
					newh = ceil(origh * factor);
			m_assert(neww > 5 && newh > 5);
			Mat32f resized(newh, neww, 3);
			resize(mat, resized);
			pyramids.emplace_back(resized, nscale);
		}
	}
}

Mat32f DOGSpace::diff(const Mat32f& img1, const Mat32f& img2) const {
	int w = img1.width(), h = img1.height();
	m_assert(w == img2.width() && h == img2.height());
	Mat32f ret(h, w, 1);
	REP(i, h) {
		// speed up
		const float *p1 = img1.ptr(i),
					      *p2 = img2.ptr(i);
		float* p = ret.ptr(i);
		REP(j, w)
			p[j] = fabs(p1[j] - p2[j]);
	}
	return ret;
}

DOGSpace::DOGSpace(ScaleSpace& ss):
	noctave(ss.noctave), nscale(ss.nscale),
	origw(ss.origw), origh(ss.origh),
	dogs(noctave)
{
#pragma omp parallel for schedule(dynamic)
	REP(i, noctave) {
		auto& o = ss.pyramids[i];
		int ns = o.get_len();
		REP(j, ns - 1)
			dogs[i].emplace_back(diff(o.get(j), o.get(j+1)));
	}
}

}
