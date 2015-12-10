//File: imgproc.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "imgproc.hh"

#include <vector>

#include <Eigen/Dense>

#include "debugutils.hh"
#include "common.hh"
#include "matrix.hh"
#include "timer.hh"

using namespace std;

// hconcat using largest height and zero padding
Mat32f hconcat(const list<Mat32f>& mats) {
	int wsum = 0;
	int hmax = 0;
	for (auto& m: mats) {
		wsum += m.width();
		update_max(hmax, m.height());
	}
	int channel = mats.front().channels();

	Mat32f ret(hmax, wsum, channel);
	fill(ret, Color::BLACK);
	wsum = 0;
	for (auto & m : mats) {
		m_assert(m.channels() == channel);
		REP(i, m.height()) {
			const float* src = m.ptr(i);
			float* dst = ret.ptr(i, wsum);
			memcpy(dst, src, m.width() * channel * sizeof(float));
		}
		wsum += m.width();
	}
	return ret;
}

// vconcat using largest width and zero padding
Mat32f vconcat(const list<Mat32f>& mats) {
	int hsum = 0;
	int wmax = 0;
	for (auto& m: mats) {
		hsum += m.height();
		update_max(wmax, m.width());
	}
	int channel = mats.front().channels();

	Mat32f ret(hsum, wmax, channel);
	fill(ret, Color::BLACK);
	hsum = 0;
	for (auto & m : mats) {
		m_assert(m.channels() == channel);
		REP(i, m.height()) {
			const float* src = m.ptr(i);
			float* dst = ret.ptr(hsum + i, 0);
			memcpy(dst, src, m.width() * channel * sizeof(float));
		}
		hsum += m.height();
	}
	return ret;
}

Color interpolate(const Mat32f& mat, float r, float c) {
	m_assert(mat.channels() == 3);
	int fr = floor(r), fc =  floor(c);
	m_assert(fr >= 0 && fr < mat.rows());
	m_assert(fc >= 0 && fc < mat.cols());
	Color ret = Color::BLACK;
	r -= fr, c -= fc;

	if (fr == mat.rows() - 1)
		fr --;
	if (fc == mat.cols() - 1)
		fc --;
	const float* p = mat.ptr(fr, fc);
	ret += Color(p) * ((1 - r) * (1 - c));
	p = mat.ptr(fr + 1, fc);
	ret += Color(p) * (r * (1 - c));
	p = mat.ptr(fr + 1, fc + 1);
	ret += Color(p) * (r * c);
	p = mat.ptr(fr, fc + 1);
	ret += Color(p[0], p[1], p[2]) * ((1 - r) * c);
	return ret;
}

bool is_edge_color(const Mat32f& mat, float y, float x) {
	int w = mat.width(), h = mat.height();
	if (!between(x, 0, w) || !between(y, 0, h)) return true;
	m_assert(mat.channels() == 3);
	int fx = floor(x), fy = floor(y);

	if(fy+1<h && fx+6<w) {
		const float* ptr = mat.ptr(fy, fx);
		REP(i, 6) if (ptr[i] < 0) return true;
		ptr = mat.ptr(fy + 1, fx);
		REP(i, 6) if (ptr[i] < 0) return true;
	}
	return false;
}

void fill(Mat32f& mat, const Color& c) {
	float* ptr = mat.ptr();
	int n = mat.pixels();
	REP(i, n) {
		c.write_to(ptr);
		ptr += 3;
	}
}

Mat32f crop(const Mat32f& mat) {
	int w = mat.width(), h = mat.height();
	int *height=new int[w], *left=new int[w], *right=new int[w];
	int maxarea = 0;
	int ll = 0, rr = 0, hh = 0, nl = 0;
	memset(height, 0, sizeof(height));
	REP(line, h) {
		REP(k, w) {
			const float* p = mat.ptr(line, k);
			float m = max(max(p[0], p[1]), p[2]);
			height[k] = m < 0 ? 0 : height[k] + 1;	// judge Color::NO
		}

		REP(k, w) {
			left[k] = k;
			while (left[k] > 0 && height[k] <= height[left[k] - 1])
				left[k] = left[left[k] - 1];
		}
		REPD(k, w - 1, 0) {
			right[k] = k;
			while (right[k] < w - 1 && height[k] <= height[right[k] + 1])
				right[k] = right[right[k] + 1];
		}
		REP(k, w)
			if (update_max(maxarea, (right[k] - left[k] + 1) * height[k]))
				ll = left[k], rr = right[k], hh = height[k], nl = line;
	}
	Mat32f ret(hh, rr - ll + 1, 3);
	int offsetx = ll, offsety = nl - hh + 1;
	REP(i, ret.height()) {
		float* dst = ret.ptr(i, 0);
		const float* src = mat.ptr(i + offsety, offsetx);
		memcpy(dst, src, 3 * ret.width() * sizeof(float));
	}

  delete[] height;
  delete[] left;
  delete[] right;
	return ret;
}

Mat32f rgb2grey(const Mat32f& mat) {
	Mat32f ret(mat.height(), mat.width(), 1);
	const float* src = mat.ptr();
	float* dst = ret.ptr();
	int n = mat.pixels();
	int idx = 0;
	for (int i = 0; i < n; ++i) {
		dst[i] = (src[idx] + src[idx+1] + src[idx+2]) / 3.f;
		idx += 3;
	}
	return ret;
}

Matrix getPerspectiveTransform(const std::vector<Vec2D>& p1, const std::vector<Vec2D>& p2) {
	using namespace Eigen;
	int n = p1.size();
	m_assert(n == (int)p2.size() && n >= 4);

	// solve with constraint h(2,2) = 1
	MatrixXd m(n * 2, 8);
	VectorXd b(n * 2);
	REP(i, n) {
		const Vec2D &m0 = p1[i], &m1 = p2[i];
		m.row(i) << m1.x, m1.y, 1, 0, 0, 0, -m1.x * m0.x, -m1.y * m0.x;
		b(i, 0) = m0.x;

		m.row(n + i) << 0, 0, 0, m1.x, m1.y, 1, -m1.x * m0.y, -m1.y * m0.y;
		b(n + i, 0) = m0.y;
	}
	VectorXd ans = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	::Matrix ret(3, 3);
	REP(i, 8) ret.ptr()[i] = ans[i];
	ret.at(2, 2) = 1;

	/* // solve with constraint |h| = 1
	 *MatrixXd m(n*2, 9);
	 *REP(i, n) {
	 *  const Vec2D &m0 = p1[i], &m1 = p2[i];
	 *  m.row(i) << m1.x, m1.y, 1, 0, 0, 0, -m1.x * m0.x, -m1.y * m0.x, -m0.x;
	 *  m.row(n + i) << 0, 0, 0, m1.x, m1.y, 1, -m1.x * m0.y, -m1.y * m0.y, -m0.y;
	 *}
	 *VectorXd ans = m.jacobiSvd(ComputeThinU | ComputeThinV).matrixV().col(8);
	 *::Matrix ret(3, 3);
	 *REP(i, 9) ret.ptr()[i] = ans(i) / ans(8);
	 */
	return ret;
}

Matrix getAffineTransform(const std::vector<Vec2D>& p1, const std::vector<Vec2D>& p2) {
	using namespace Eigen;
	int n = p1.size();
	m_assert(n == (int)p2.size() && n >= 3);

	MatrixXd m(n * 2, 6);
	VectorXd b(n * 2);
	REP(i, n) {
		const Vec2D &m0 = p1[i], &m1 = p2[i];
		m.row(i * 2) << m1.x, m1.y, 1, 0, 0, 0;
		b(i * 2, 0) = m0.x;
		m.row(i * 2 + 1) << 0, 0, 0, m1.x, m1.y, 1;
		b(i * 2 + 1, 0) = m0.y;
	}

	VectorXd ans = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	::Matrix ret(3, 3); ret.zero();
	REP(i, 6) ret.ptr()[i] = ans[i];
	ret.at(2, 2) = 1;
	return ret;
}

namespace {

	void resize_bilinear(const Mat32f &src, Mat32f &dst) {
		vector<int> tabsx(dst.rows());
		vector<int> tabsy(dst.cols());
		vector<float> tabrx(dst.rows());
		vector<float> tabry(dst.cols());

		const float fx = (float)(dst.rows()) / src.rows();
		const float fy = (float)(dst.cols()) / src.cols();
		const float ifx = 1.f / fx;
		const float ify = 1.f / fy;
		for (int dx = 0; dx < dst.rows(); ++dx) {
			float rx = (dx+0.5f) * ifx - 0.5f;
			int sx = floor(rx);
			rx -= sx;
			if (sx < 0) {
				sx = rx = 0;
			} else if (sx + 1 >= src.rows()) {
				sx = src.rows() - 2;
				rx = 1;
			}
			tabsx[dx] = sx;
			tabrx[dx] = rx;
		}
		for (int dy = 0; dy < dst.cols(); ++dy) {
			float ry = (dy+0.5f) * ify - 0.5f;
			int sy = floor(ry);
			ry -= sy;
			if (sy < 0) {
				sy = ry = 0;
				ry = 0;
			} else if (sy + 1 >= src.cols()) {
				sy = src.cols() - 2;
				ry = 1;
			}
			tabsy[dy] = sy;
			tabry[dy] = ry;
		}

		const int ch = src.channels();
		for (int dx = 0; dx < dst.rows(); ++dx) {
			const float *p0 = src.ptr(tabsx[dx]+0);
			const float *p1 = src.ptr(tabsx[dx]+1);
			float *pdst = dst.ptr(dx);
			float rx = tabrx[dx], irx = 1.0f - rx;
			for (int dy = 0; dy < dst.cols(); ++dy) {
				float *pcdst = pdst + dy*ch;
				const float *pc00 = p0 + (tabsy[dy]+0)*ch;
				const float *pc01 = p0 + (tabsy[dy]+1)*ch;
				const float *pc10 = p1 + (tabsy[dy]+0)*ch;
				const float *pc11 = p1 + (tabsy[dy]+1)*ch;
				float ry = tabry[dy], iry = 1.0f - ry;
				for (int c = 0; c < ch; ++c) {
					float res = rx * (pc11[c]*ry + pc10[c]*iry)
						+ irx * (pc01[c]*ry + pc00[c]*iry);
					pcdst[c] = res;
				}
			}
		}
	}
}	// namespace

template <>
void resize<float>(const Mat32f &src, Mat32f &dst) {
	m_assert(src.rows() > 1);
	m_assert(src.cols() > 1);
	m_assert(dst.rows() > 1);
	m_assert(dst.cols() > 1);
	m_assert(src.channels() == dst.channels());
	m_assert(src.channels() == 1 || src.channels() == 3);
	return resize_bilinear(src, dst);
}
