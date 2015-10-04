// File: transformer.cc
// Date: Fri May 03 23:04:58 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "transformer.hh"
#include "lib/config.hh"
#include "lib/timer.hh"
#include <set>
using namespace std;

// TODO find out when not matched
// get a transform matix from second -> first
bool TransFormer::get_transform(Homography* ret) {
	TotalTimer tm("get_transform");
	int REQUIRED = (HOMO ? HOMO_FREEDOM: AFFINE_FREEDOM) + 1 / 2;
	int n_match = match.size();
	if (n_match < REQUIRED) {
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
		REP(_, REQUIRED) {
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
	if (maxinlierscnt <= 10) {
		cerr << "Transform failed: cannot find a proper matrix." << endl;
		return false;
	}
	inliers = get_inliers(best_transform);
	best_transform = calc_transform(inliers);

	print_debug("final inlier size: %lu\n", inliers.size());
	*ret = best_transform;
	return true;
}

Homography TransFormer::calc_transform(const vector<int>& matches) const {
	TotalTimer tm("calc_transform");
	if (HOMO)
		return calc_homo_transform(matches);
	else
		return calc_affine_transform(matches);
}

Homography TransFormer::calc_affine_transform(const vector<int>& matches) const {
	int n = matches.size();
	m_assert(n * 2 >= AFFINE_FREEDOM);

	Matrix m(2 * n, AFFINE_FREEDOM); m.zero();
	Matrix b(2 * n, 1);
	REP(i, n) {
		const Vec2D &m0 = f1[match.data[matches[i]].x].coor,	// rhs
								&m1 = f2[match.data[matches[i]].y].coor;	// lhs
		m.at(i * 2, 0) = m1.x; m.at(i * 2, 1) = m1.y; m.at(i * 2, 2) = 1;
		b.at(i * 2, 0) = m0.x;

		m.at(i * 2 + 1, 3) = m1.x; m.at(i * 2 + 1, 4) = m1.y; m.at(i * 2 + 1, 5) = 1;
		b.at(i * 2 + 1, 0) = m0.y;
	}
	Matrix res = m.solve_overdetermined(b);
	Homography ret; ret.zero();
	memcpy(ret.ptr(), res.ptr(), AFFINE_FREEDOM * sizeof(double));
	ret.at(2, 2) = 1.0;
	return move(ret);
}

Homography TransFormer::calc_homo_transform(const vector<int>& matches) const {
	int n = matches.size();
	m_assert(n * 2 >= HOMO_FREEDOM);

	Matrix m(2 * n, HOMO_FREEDOM); m.zero();
	Matrix b(2 * n, 1);
	REP(i, n) {
		const Vec2D &m0 = f1[match.data[matches[i]].x].coor,	//rhs
								&m1 = f2[match.data[matches[i]].y].coor;  //lhs
		m.at(i * 2, 0) = m1.x; m.at(i * 2, 1) = m1.y; m.at(i * 2, 2) = 1;
		m.at(i * 2, 6) = -m1.x * m0.x; m.at(i * 2, 7) = -m1.y * m0.x;
		b.at(i * 2, 0) = m0.x;

		m.at(i * 2 + 1, 3) = m1.x; m.at(i * 2 + 1, 4) = m1.y; m.at(i * 2 + 1, 5) = 1;
		m.at(i * 2 + 1, 6) = -m1.x * m0.y; m.at(i * 2 + 1, 7) = -m1.y * m0.y;
		b.at(i * 2 + 1, 0) = m0.y;
	}
	Matrix res = m.solve_overdetermined(b);
	Homography ret;
	memcpy(ret.ptr(), res.ptr(), HOMO_FREEDOM * sizeof(double));
	ret.at(2, 2) = 1;
	// check
	// for (auto &i : matches) {
	// 	Vec2D project = cal_project(ret, i.second);
	// 	cout << i.first << " == ?" << project << endl;
	// }
	return ret;
}

vector<int> TransFormer::get_inliers(const Homography& trans) const {
	static double INLIER_DIST = RANSAC_INLIER_THRES * RANSAC_INLIER_THRES;
	TotalTimer tm("get_inlier");
	vector<int> ret;
	int n = match.size();

	Matrix transformed = trans.prod(f2_homo_coor);	// 3xn
	REP(i, n) {
		const Vec2D& fcoor = f1[match.data[i].x].coor;
		double idenom = 1.f / transformed.at(2, i);
		double dist = (Vec2D(transformed.at(0, i) * idenom,
					transformed.at(1, i) * idenom) - fcoor).sqr();
		if (dist < INLIER_DIST)
			ret.push_back(i);
	}
	return ret;
}

real_t TransFormer::get_focal_from_matrix(const Matrix& m) {
	real_t f2;
	real_t p1 = sqr(m.at(0, 0)) + sqr(m.at(0, 1)) - sqr(m.at(1, 0)) - sqr(m.at(1, 1));
	if (fabs(p1) > EPS)  {
		f2 = (sqr(m.at(1, 2)) - sqr(m.at(0, 2))) / (p1);
	} else {
		p1 = m.at(0, 0) * m.at(1, 0) + m.at(0, 1) * m.at(1, 1);
		if (fabs(p1) > EPS)
			f2 = -(m.at(0, 2) * m.at(1, 2)) / p1;
		else {
			return Vec(m.at(0, 0), m.at(0, 1), m.at(0, 2)).dot(Vec(m.at(2, 0), m.at(2, 1), m.at(2, 2)));
		}
	}
	return sqrt(fabs(f2));
}
