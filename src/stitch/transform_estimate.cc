// File: transform_estimate.cc
// Date: Fri May 03 23:04:58 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "transform_estimate.hh"

#include <set>
#include <random>

#include "feature/feature.hh"
#include "feature/matcher.hh"
#include "lib/config.hh"
#include "lib/imgproc.hh"
#include "lib/timer.hh"
#include "match_info.hh"
using namespace std;
using namespace config;

namespace {
const int ESTIMATE_MIN_NR_MATCH = 6;
}

namespace pano {

TransformEstimation::TransformEstimation(const MatchData& m_match,
		const std::vector<Descriptor>& m_f1,
		const std::vector<Descriptor>& m_f2,
		const Shape2D& shape1):
	match(m_match), f1(m_f1), f2(m_f2),
	f2_homo_coor(3, match.size())
{
	if (CYLINDER || TRANS)
		transform_type = Affine;
	else
		transform_type = Homo;
	int n = match.size();
	if (n < ESTIMATE_MIN_NR_MATCH) return;
	REP(i, n) {
		Vec2D old = f2[match.data[i].second].coor;
		f2_homo_coor.at(0, i) = old.x;
		f2_homo_coor.at(1, i) = old.y;
		f2_homo_coor.at(2, i) = 1;
	}
	ransac_inlier_thres = (shape1.w + shape1.h) * 0.5 / 800 * RANSAC_INLIER_THRES;
}

bool TransformEstimation::get_transform(MatchInfo* info) {
	TotalTimer tm("get_transform");
	// use Affine in cylinder mode, and Homography in normal mode
	int nr_match_used = (transform_type == Affine ? 6: 8) + 1 / 2;
	int nr_match = match.size();
	if (nr_match < nr_match_used)
		return false;

	vector<int> inliers;
	set<int> selected;

	int maxinlierscnt = -1;
	Matrix best_transform(3, 3);

	random_device rd;
	mt19937 rng(rd());

	for (int K = RANSAC_ITERATIONS; K --;) {
		inliers.clear();
		selected.clear();
		REP(_, nr_match_used) {
			int random;
			do {
				random = rng() % nr_match;
			} while (selected.find(random) != selected.end());
			selected.insert(random);
			inliers.push_back(random);
		}
		Matrix transform = calc_transform(inliers);
		if (! Homography::health(transform.ptr()))
			continue;
		int n_inlier = get_inliers(transform).size();
		if (update_max(maxinlierscnt, n_inlier))
			best_transform = move(transform);
	}
	inliers = get_inliers(best_transform);
	if (inliers.size() <= 8) {
		info->confidence = -(float)inliers.size();	// for debug
		return false;
	}
	best_transform = calc_transform(inliers);
	fill_inliers_to_matchinfo(inliers, info);
	info->homo = best_transform;
	return true;
}

Matrix TransformEstimation::calc_transform(const vector<int>& matches) const {
	int n = matches.size();
	vector<Vec2D> p1, p2;
	REP(i, n) {
		p1.emplace_back(f1[match.data[matches[i]].first].coor);
		p2.emplace_back(f2[match.data[matches[i]].second].coor);
	}
	if (transform_type == Affine)
		return getAffineTransform(p1, p2);
	else
		return getPerspectiveTransform(p1, p2);
}

vector<int> TransformEstimation::get_inliers(const Matrix& trans) const {
	float INLIER_DIST = sqr(ransac_inlier_thres);
	TotalTimer tm("get_inlier");
	vector<int> ret;
	int n = match.size();

	Matrix transformed = trans.prod(f2_homo_coor);	// 3xn
	REP(i, n) {
		const Vec2D& fcoor = f1[match.data[i].first].coor;
		double idenom = 1.f / transformed.at(2, i);
		double dist = (Vec2D(transformed.at(0, i) * idenom,
					transformed.at(1, i) * idenom) - fcoor).sqr();
		if (dist < INLIER_DIST)
			ret.push_back(i);
	}
	return ret;
}

bool TransformEstimation::good_inlier_set(const std::vector<int>& inliers) const {
	vector<Vec2D> coor1, coor2;
	for (auto& idx: inliers) {
		coor1.emplace_back(f1[match.data[idx].first].coor);
		coor2.emplace_back(f2[match.data[idx].second].coor);
	}

	return true;
}

void TransformEstimation::fill_inliers_to_matchinfo(
		const std::vector<int>& inliers, MatchInfo* info) const {
	info->match.clear();
	for (auto& idx : inliers) {
		info->match.emplace_back(
				f1[match.data[idx].first].coor,
				f2[match.data[idx].second].coor
				);
	}
	// From D. Lowe 2008 Automatic Panoramic Image Stitching
	// TODO filter out low confidence, by finding the overlapping area
	info->confidence = inliers.size() / (8 + 0.3 * match.size());

	// overlap too much. not helpful. but might need to keep it for connectivity
	if (info->confidence > 3.1) {
		info->confidence = 0.;
		print_debug("If you are not giving two almost identical image, then there is a bug..\n");
	}
}

}
