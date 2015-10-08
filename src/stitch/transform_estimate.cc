// File: transform_estimate.cc
// Date: Fri May 03 23:04:58 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "transform_estimate.hh"

#include <set>
#include <Eigen/Dense>

#include "match_info.hh"
#include "lib/config.hh"
#include "lib/timer.hh"
#include "feature/feature.hh"
#include "feature/matcher.hh"
using namespace std;
using namespace feature;

TransformEstimation::TransformEstimation(const feature::MatchData& m_match,
		const std::vector<feature::Descriptor>& m_f1,
		const std::vector<feature::Descriptor>& m_f2):
	match(m_match), f1(m_f1), f2(m_f2),
	f2_homo_coor(3, match.size())
{
	int n = match.size();
	if (n < 6) return;
	REP(i, n) {
		Vec2D old = f2[match.data[i].second].coor;
		f2_homo_coor.at(0, i) = old.x;
		f2_homo_coor.at(1, i) = old.y;
	}
	REP(i, n) f2_homo_coor.at(2, i) = 1;
}

// get a transform matix from second -> first
bool TransformEstimation::get_transform(MatchInfo* info) {
	TotalTimer tm("get_transform");
	int nr_match_used = (HOMO ? HOMO_FREEDOM: AFFINE_FREEDOM) + 1 / 2;
	int n_match = match.size();
	if (n_match < 6) {
		cerr << "Transform failed: only have " << n_match << " feature matches." << endl;
		return false;
	}

	vector<int> inliers;
	set<int> selected;

	int maxinlierscnt = -1;
	Homography best_transform;

	for (int K = RANSAC_ITERATIONS; K --;) {
		inliers.clear();
		selected.clear();
		REP(_, nr_match_used) {
			int random;
			do {
				random = rand() % n_match;
			} while (selected.find(random) != selected.end());
			selected.insert(random);
			inliers.push_back(random);
		}
		Homography transform(calc_transform(inliers));
		int n_inlier = get_inliers(transform).size();
		if (update_max(maxinlierscnt, n_inlier)) {
			best_transform = move(transform);
		}
	}
	inliers = get_inliers(best_transform);
	if (inliers.size() <= 10)
		return false;
	best_transform = calc_transform(inliers);
	fill_inliers_to_matchinfo(inliers, info);
	info->homo = best_transform;
	return true;
}

Homography TransformEstimation::calc_transform(const vector<int>& matches) const {
	if (HOMO)
		return calc_homo_transform(matches);
	else
		return calc_affine_transform(matches);
}

Homography TransformEstimation::calc_affine_transform(const vector<int>& matches) const {
	using namespace Eigen;
	int n = matches.size();
	m_assert(n * 2 >= AFFINE_FREEDOM);

	MatrixXd m(n * 2, AFFINE_FREEDOM);
	VectorXd b(n * 2);
	REP(i, n) {
		const Vec2D &m0 = f1[match.data[matches[i]].first].coor,	// rhs
					&m1 = f2[match.data[matches[i]].second].coor;	// lhs
		m.row(i * 2) << m1.x, m1.y, 1, 0, 0, 0;
		b(i * 2, 0) = m0.x;

		m.row(i * 2 + 1) << 0, 0, 0, m1.x, m1.y, 1;
		b(i * 2 + 1, 0) = m0.y;
	}
	VectorXd ans = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	Homography ret; ret.zero();
	REP(i, AFFINE_FREEDOM) ret.ptr()[i] = ans[i];
	ret.at(2, 2) = 1.0;
	return move(ret);
}

Homography TransformEstimation::calc_homo_transform(const vector<int>& matches) const {
	using namespace Eigen;
	int n = matches.size();
	m_assert(n * 2 >= HOMO_FREEDOM);

	MatrixXd m(n * 2, HOMO_FREEDOM);
	VectorXd b(n * 2);
	REP(i, n) {
		const Vec2D &m0 = f1[match.data[matches[i]].first].coor,	//rhs
					&m1 = f2[match.data[matches[i]].second].coor;  //lhs
		m.row(i * 2) << m1.x, m1.y, 1, 0, 0, 0, -m1.x * m0.x, -m1.y * m0.y;
		b(i * 2, 0) = m0.x;

		m.row(i * 2 + 1) << 0, 0, 0, m1.x, m1.y, 1, -m1.x * m0.y, -m1.y * m0.y;
		b(i * 2 + 1, 0) = m0.y;
	}

	VectorXd ans = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
	Homography ret;
	REP(i, HOMO_FREEDOM) ret.ptr()[i] = ans[i];
	ret.at(2, 2) = 1;
	// check
	// for (auto &i : matches) {
	// 	Vec2D project = cal_project(ret, i.second);
	// 	cout << i.first << " == ?" << project << endl;
	// }
	return ret;
}

vector<int> TransformEstimation::get_inliers(const Homography& trans) const {
	static double INLIER_DIST = RANSAC_INLIER_THRES * RANSAC_INLIER_THRES;
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

void TransformEstimation::fill_inliers_to_matchinfo(
		const std::vector<int>& inliers, MatchInfo* info) const {
	info->match.clear();
	for (auto& idx : inliers) {
		info->match.emplace_back(
				f1[match.data[idx].first].coor,
				f2[match.data[idx].second].coor
				);
	}
	info->confidence = inliers.size() / (8 + 0.3 * match.size());
	if (info->confidence > 3)
		info->confidence = 0.;		// overlap too much. not helpful
}

