//File: orientation.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>
#include "orientation.hh"
#include "lib/config.hh"
#include "lib/timer.hh"
#include "feature.hh"
using namespace std;
using namespace config;


namespace pano {

OrientationAssign::OrientationAssign(
		const DOGSpace& dog, const ScaleSpace& ss,
		const std::vector<SSPoint>& keypoints):
	dog(dog),ss(ss), points(keypoints) {}

vector<SSPoint> OrientationAssign::work() const {
	vector<SSPoint> ret;
	for (auto& p : points) {
		auto major_orient = calc_dir(p);
		for (auto& o : major_orient) {
			ret.emplace_back(p);
			ret.back().dir = o;
		}
	}
	return ret;
}

std::vector<float> OrientationAssign::calc_dir(
		const SSPoint& p) const {
	const static float halfipi = 0.5f / M_PI;
	auto& pyramid = ss.pyramids[p.pyr_id];
	auto& orient_img = pyramid.get_ort(p.scale_id);
	auto& mag_img = pyramid.get_mag(p.scale_id);

	float gauss_weight_sigma = p.scale_factor * ORI_WINDOW_FACTOR;
	int rad = round(p.scale_factor * ORI_RADIUS);
	float exp_denom = 2 * sqr(gauss_weight_sigma);
	float hist[ORI_HIST_BIN_NUM];
	memset(hist, 0, sizeof(hist));

	// calculate gaussian/magnitude weighted histogram
	// of orientation inside a circle
	for (int xx = -rad; xx < rad; xx ++) {
		int newx = p.coor.x + xx;
		// because mag/ort on the border is zero
		if (! between(newx, 1, pyramid.w - 1)) continue;
		for (int yy = -rad; yy < rad; yy ++) {
			int newy = p.coor.y + yy;
			if (! between(newy, 1, pyramid.h - 1)) continue;
		  // use a circular gaussian window
			if (sqr(xx) + sqr(yy) > sqr(rad)) continue;
			float orient = orient_img.at(newy, newx);
			int bin = round(ORI_HIST_BIN_NUM * halfipi * orient );
			if (bin == ORI_HIST_BIN_NUM) bin = 0;
			m_assert(bin < ORI_HIST_BIN_NUM);

			float weight = expf(-(sqr(xx) + sqr(yy)) / exp_denom);
			hist[bin] += weight * mag_img.at(newy, newx);
		}
	}

	// TODO do we need this?
	// smooth the histogram by interpolation
	for (int K = ORI_HIST_SMOOTH_COUNT; K --;)
		REP(i, ORI_HIST_BIN_NUM) {
			float prev = hist[i == 0 ? ORI_HIST_BIN_NUM - 1 : i - 1];
			float next = hist[i == ORI_HIST_BIN_NUM - 1 ? 0 : i + 1];
			hist[i] = hist[i] * 0.5 + (prev + next) * 0.25;
		}

	float maxbin = 0;
	for (auto i : hist) update_max(maxbin, i);
	float thres = maxbin * ORI_HIST_PEAK_RATIO;
	vector<float> ret;

	REP(i, ORI_HIST_BIN_NUM) {
		float prev = hist[i == 0 ? ORI_HIST_BIN_NUM - 1 : i - 1];
		float next = hist[i == ORI_HIST_BIN_NUM - 1 ? 0 : i + 1];
		// choose extreme orientation which is larger than thres
		if (hist[i] > thres && hist[i] > max(prev, next)) {
			// parabola interpolation
			real_t newbin = (float)i - 0.5 + (hist[i] - prev) / (prev + next - 2 * hist[i]);
			// I saw this elsewhere... although by derivation the above one should be correct
			//real_t newbin = (float)i - 0.5 + (next - prev) / (prev + next - 2 * hist[i]);
			if (newbin < 0)
				newbin += ORI_HIST_BIN_NUM;
			else if (newbin >= ORI_HIST_BIN_NUM)
				newbin -= ORI_HIST_BIN_NUM;
			float ort = newbin / ORI_HIST_BIN_NUM * 2 * M_PI;
			ret.push_back(ort);
		}
	}
	return ret;
}

}
