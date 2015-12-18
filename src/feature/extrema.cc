//File: extrema.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "extrema.hh"
#include "lib/config.hh"
#include "lib/matrix.hh"
#include "lib/timer.hh"
#include "feature.hh"
#include <vector>
using namespace std;
using namespace config;

namespace pano {

ExtremaDetector::ExtremaDetector(const DOGSpace& dg):
	dog(dg) {}

vector<Coor> ExtremaDetector::get_raw_extrema() const {
	vector<Coor> ret;
	int npyramid = dog.noctave, nscale = dog.nscale;
	REP(i, npyramid)
		REPL(j, 1, nscale - 2) {
			auto& now = dog.dogs[i][j];
			int w = now.width(), h = now.height();

			auto v = get_local_raw_extrema(i, j);
			for (auto& c : v) {
				ret.emplace_back((float)c.x / w * dog.origw,
						(float)c.y / h * dog.origh);
			}
		}
	return ret;
}

vector<SSPoint> ExtremaDetector::get_extrema() const {
	TotalTimer tm("extrema");
	int npyramid = dog.noctave, nscale = dog.nscale;
	vector<SSPoint> ret;
#pragma omp parallel for schedule(dynamic)
	REP(i, npyramid)
		REPL(j, 1, nscale - 2) {
			auto v = get_local_raw_extrema(i, j);
			//print_debug("raw extrema count: %lu\n", v.size());
			for (auto& c : v) {
				SSPoint sp;
				sp.coor = c;
				sp.pyr_id = i;
				sp.scale_id = j;
				bool succ = calc_kp_offset(&sp);
				if (! succ) continue;
				auto& img = dog.dogs[i][sp.scale_id];
				succ = ! is_edge_response(sp.coor, img);
				if (! succ) continue;

#pragma omp critical
				ret.emplace_back(sp);
			}
		}
	return ret;
}

bool ExtremaDetector::calc_kp_offset(SSPoint* sp) const {
	auto& now_pyramid = dog.dogs[sp->pyr_id];
	auto& now_img = now_pyramid[sp->scale_id];
	int w = now_img.width(), h = now_img.height();
	int nscale = dog.nscale;

	Vec offset, delta;	// partial(d) / partial(offset)
	int nowx = sp->coor.x, nowy = sp->coor.y, nows = sp->scale_id;
	int niter = 0;
	for(;niter < CALC_OFFSET_DEPTH; ++niter) {
		if (!between(nowx, 1, w - 1) ||
				!between(nowy, 1, h - 1) ||
				!between(nows, 1, nscale - 2))
			return false;

		auto iter_offset = calc_kp_offset_iter(
				now_pyramid, nowx, nowy, nows);
		offset = iter_offset.first;
		delta = iter_offset.second;
		if (offset.get_abs_max() < OFFSET_THRES) // found
			break;

		nowx += round(offset.x);
		nowy += round(offset.y);
		nows += round(offset.z);
	}
	if (niter == CALC_OFFSET_DEPTH) return false;

	double dextr = offset.dot(delta);		// calc D(x~)
	dextr = now_pyramid[nows].at(nowy, nowx) + dextr / 2;
	// contrast too low
	if (dextr < CONTRAST_THRES) return false;

	// update the point
	sp->coor = Coor(nowx, nowy);
	sp->scale_id = nows;
	sp->scale_factor = GAUSS_SIGMA * pow(
				SCALE_FACTOR, ((double)nows + offset.z) / nscale);
	// accurate real-value coor
	sp->real_coor = Vec2D(
			((double)nowx + offset.x) / w,
			((double)nowy + offset.y) / h);
	return true;
}

std::pair<Vec, Vec> ExtremaDetector::calc_kp_offset_iter(
		const DOGSpace::DOG& now_pyramid,
		int x , int y, int s) const {
	Vec offset = Vec::get_zero();
	Vec delta;
	double dxx, dyy, dss, dxy, dys, dsx;

	auto& now_scale = now_pyramid[s];
#define D(x, y, s) now_pyramid[s].at(y, x)
#define DS(x, y) now_scale.at(y, x)
	float val = DS(x, y);

	delta.x = (DS(x + 1, y) - DS(x - 1, y)) / 2;
	delta.y = (DS(x, y + 1) - DS(x, y - 1)) / 2;
	delta.z = (D(x, y, s + 1) - D(x, y, s - 1)) / 2;

	dxx = DS(x + 1, y) + DS(x - 1, y) - val - val;
	dyy = DS(x, y + 1) + DS(x, y - 1) - val - val;
	dss = D(x, y, s + 1) + D(x, y, s - 1) - val - val;

	dxy = (DS(x + 1, y + 1) - DS(x + 1, y - 1) - DS(x - 1, y + 1) + DS(x - 1, y - 1)) / 4;
	dys = (D(x, y + 1, s + 1) - D(x, y - 1, s + 1) - D(x, y + 1, s - 1) + D(x, y - 1, s - 1)) / 4;
	dsx = (D(x + 1, y, s + 1) - D(x - 1, y, s + 1) - D(x + 1, y, s - 1) + D(x - 1, y, s - 1)) / 4;
#undef D
#undef DS

	Matrix m(3, 3);
	m.at(0, 0) = dxx; m.at(1, 1) = dyy; m.at(2, 2) = dss;
	m.at(0, 1) = m.at(1, 0) = dxy;
	m.at(0, 2) = m.at(2, 0) = dsx;
	m.at(1, 2) = m.at(2, 1) = dys;

	Matrix pdpx(3, 1);	// delta = dD / dx
	delta.write_to(pdpx.ptr());

	Matrix inv;
	if (! m.inverse(inv)) {	  // pseudo inverse is slow
		inv = m.pseudo_inverse();
	}
	auto prod = inv.prod(pdpx);
	offset = Vec(prod.ptr());
	return {offset, delta};
}

bool ExtremaDetector::is_edge_response(Coor coor, const Mat32f& img) const {
	float dxx, dxy, dyy;
	int x = coor.x, y = coor.y;
	float val = img.at(y, x);

	dxx = img.at(y, x + 1) + img.at(y, x - 1) - val - val;
	dyy = img.at(y + 1, x) + img.at(y - 1, x) - val - val;
	dxy = (img.at(y + 1, x + 1) + img.at(y - 1, x - 1) -
			img.at(y + 1, x - 1) - img.at(y - 1, x + 1)) / 4;
	float det = dxx * dyy - dxy * dxy;
	if (det <= 0) return true;
	float tr2 = sqr(dxx + dyy);

	// Calculate principal curvature by hessian
	if (tr2 / det < sqr(EDGE_RATIO + 1) / EDGE_RATIO) return false;
	return true;
}

vector<Coor> ExtremaDetector::get_local_raw_extrema(
		int pyr_id, int scale_id) const {
	vector<Coor> ret;

	const Mat32f& now(dog.dogs[pyr_id][scale_id]);
	int w = now.width(), h = now.height();

	auto is_extrema = [this, &now, pyr_id, scale_id](int r, int c) {
		float center = now.at(r, c);
		if (center < PRE_COLOR_THRES)			// initial color is less than thres
			return false;

		bool max = true, min = true;
		float cmp1 = center - JUDGE_EXTREMA_DIFF_THRES,
					cmp2 = center + JUDGE_EXTREMA_DIFF_THRES;
		// try same scale
		REPL(di, -1, 2) REPL(dj, -1, 2) {
			if (!di && !dj) continue;
			float newval = now.at(r + di, c + dj);
			if (newval >= cmp1) max = false;
			if (newval <= cmp2) min = false;
			if (!max && !min) return false;
		}

		// try adjencent scale
		for (int ds = -1; ds < 2; ds += 2) {
			int nl = scale_id + ds;
			auto& mat = this->dog.dogs[pyr_id][nl];

			REPL(di, -1, 2) {
				const float* p = mat.ptr(r + di) + c - 1;
				REP(i, 3) {
					float newval = p[i];
					if (newval >= cmp1) max = false;
					if (newval <= cmp2) min = false;
					if (!max && !min) return false;
				}
			}
		}
		return true;
	};

	REPL(i, 1, h - 1) REPL(j, 1, w - 1)
		if (is_extrema(i, j))
			ret.emplace_back(j, i);
	return ret;
}

}
