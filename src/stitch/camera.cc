//File: camera.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "camera.hh"
#include "transform.hh"
#include <algorithm>
using namespace std;
// implement stuffs about matrices

namespace {

// Creating Full View Panoramic Image Mosaics - Szeliski
double get_focal_from_matrix(const Matrix& m) {
	const double* h = m.ptr();

	double d1, d2; // Denominators
	double v1, v2; // Focal squares value candidates
	double f1, f0;
	bool f1_ok, f0_ok;

	f1_ok = true;
	d1 = h[6] * h[7];
	d2 = (h[7] - h[6]) * (h[7] + h[6]);
	v1 = -(h[0] * h[1] + h[3] * h[4]) / d1;
	v2 = (h[0] * h[0] + h[3] * h[3] - h[1] * h[1] - h[4] * h[4]) / d2;
	if (v1 < v2) swap(v1, v2);
	if (v1 > 0 && v2 > 0) f1 = sqrt(abs(d1) > abs(d2) ? v1 : v2);
	else if (v1 > 0) f1 = sqrt(v1);
	else f1_ok = false;

	f0_ok = true;
	d1 = h[0] * h[3] + h[1] * h[4];
	d2 = h[0] * h[0] + h[1] * h[1] - h[3] * h[3] - h[4] * h[4];
	v1 = -h[2] * h[5] / d1;
	v2 = (h[5] * h[5] - h[2] * h[2]) / d2;
	if (v1 < v2) swap(v1, v2);
	if (v1 > 0 && v2 > 0) f0 = sqrt(abs(d1) > abs(d2) ? v1 : v2);
	else if (v1 > 0) f0 = sqrt(v1);
	else f0_ok = false;
	if (f1_ok && f0_ok) return sqrt(f1 * f0);
	return 0;
}

}

Camera::Camera() : R(Homography::I()), t(0,0,0) { }

Homography Camera::K() const {
	Homography ret(Homography::I());
	ret.at(0,0) = focal;
	ret.at(0,2) = ppx;
	ret.at(1,1) = focal * aspect;
	ret.at(1,2) = ppy;
	return ret;
}

double Camera::estimate_focal(
		const vector<vector<MatchInfo>>& matches) {
	int n = matches.size();
	vector<double> estimates;
	REP(i, n) REPL(j, i + 1, n) {
		auto& match = matches[i][j];
		if (match.confidence < EPS) continue;
		estimates.emplace_back(
				get_focal_from_matrix(match.homo));
	}
	int ne = estimates.size();
	if (ne < min(n - 1, 3))
		return -1;	// focal estimate fail
	sort(begin(estimates), end(estimates));
	if (ne % 2 == 1)
		return estimates[ne >> 1];
	else
		return (estimates[ne >> 1] + estimates[(ne >> 1) - 1]) * 0.5;
}
