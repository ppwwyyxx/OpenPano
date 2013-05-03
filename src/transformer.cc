// File: transformer.cc
// Date: Fri May 03 17:21:26 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "config.hh"
#include "transformer.hh"
#include <set>
using namespace std;

Matrix TransFormer::get_transform() {		// second -> first
	int REQUIRED = (HOMO ? HOMO_FREEDOM / 2 : AFFINE_FREEDOM / 2);
	int n_match = match.size();
	if (n_match < REQUIRED) {
		PP("only have matches: ", n_match);
		return Matrix(3, 3);
	}

	vector<int> fit;
	set<int> selected;

	int maxinlierscnt = -1;
	Matrix best_transform(0, 0);

	for (int K = RANSAC_ITERATIONS; K --;) {
		fit.clear();
		selected.clear();
		REP(i, REQUIRED) {
			int random;
			while (1) {
				random = rand() % n_match;
				if (selected.find(random) == selected.end())
					break;
			}
			selected.insert(random);
			fit.push_back(random);
		}
		Matrix transform = cal_transform(fit);
		int inlier = cal_inliers(transform);
		// int inlier = get_inliers(transform).size();
		if (update_max(maxinlierscnt, inlier)) {
			best_transform = move(transform);
		}
	}
	m_assert(maxinlierscnt > 0);
	auto inliers = get_inliers(best_transform);
	best_transform = cal_transform(inliers);
	inliers = get_inliers(best_transform);
	best_transform = cal_transform(inliers);
	return move(best_transform);
}

Matrix TransFormer::cal_transform(const vector<int>& matches) const {
	if (HOMO)
		return cal_homo_transform(matches);
	else
		return cal_affine_transform(matches);
	// return cal_rotate_homo_transform(matches);
}

Matrix TransFormer::cal_affine_transform(const vector<int>& matches) const {
	int n = matches.size();
	m_assert(n * 2 >= AFFINE_FREEDOM);

	Matrix m(AFFINE_FREEDOM, 2 * n);
	Matrix b(1, 2 * n);
	REP(i, n) {
		const Vec2D &m0 = f1[match.data[matches[i]].x].real_coor,
					&m1 = f2[match.data[matches[i]].y].real_coor;
		m.get(i * 2, 0) = m1.x;
		m.get(i * 2, 1) = m1.y;
		m.get(i * 2, 2) = 1;
		m.get(i * 2, 3) = m.get(i * 2, 4) = m.get(i * 2, 5) = 0;
		b.get(i * 2, 0) = m0.x;

		m.get(i * 2 + 1, 0) = m.get(i * 2 + 1, 1) = m.get(i * 2 + 1, 2) = 0;
		m.get(i * 2 + 1, 3) = m1.x;
		m.get(i * 2 + 1, 4) = m1.y;
		m.get(i * 2 + 1, 5) = 1;
		b.get(i * 2 + 1, 0) = m0.y;
	}
	Matrix res(3, 3);
	if (!m.solve_overdetermined(res, b)) { cout << "solve failed" << endl; return move(res); }
	Matrix ret(3, 3);
	REP(i, AFFINE_FREEDOM) ret.get(i / 3, i % 3) = res.get(i, 0);
	ret.get(2, 2) = 1;
	return move(ret);
}

// second -> first
Matrix TransFormer::cal_homo_transform(const vector<int>& matches) const {
	int n = matches.size();
	m_assert(n * 2 >= HOMO_FREEDOM);

	Matrix m(HOMO_FREEDOM, 2 * n);
	Matrix b(1, 2 * n);
	REP(i, n) {
		const Vec2D &m0 = f1[match.data[matches[i]].x].real_coor,
					&m1 = f2[match.data[matches[i]].y].real_coor;
		m.get(i * 2, 0) = m1.x;
		m.get(i * 2, 1) = m1.y;
		m.get(i * 2, 2) = 1;
		m.get(i * 2, 3) = m.get(i * 2, 4) = m.get(i * 2, 5) = 0;
		m.get(i * 2, 6) = -m1.x * m0.x;
		m.get(i * 2, 7) = -m1.y * m0.x;
		b.get(i * 2, 0) = m0.x;

		m.get(i * 2 + 1, 0) = m.get(i * 2 + 1, 1) = m.get(i * 2 + 1, 2) = 0;
		m.get(i * 2 + 1, 3) = m1.x;
		m.get(i * 2 + 1, 4) = m1.y;
		m.get(i * 2 + 1, 5) = 1;
		m.get(i * 2 + 1, 6) = -m1.x * m0.y;
		m.get(i * 2 + 1, 7) = -m1.y * m0.y;
		b.get(i * 2 + 1, 0) = m0.y;
	}
	Matrix res(3, 3);
	if (!m.solve_overdetermined(res, b)) { cout << "solve failed" << endl; return move(res); }
	Matrix ret(3, 3);
	REP(i, HOMO_FREEDOM) ret.get(i / 3, i % 3) = res.get(i, 0);
	ret.get(2, 2) = 1;

	// check
	// for (auto &i : matches) {
	// 	Vec2D project = cal_project(ret, i.second);
	// 	cout << i.first << " == ?" << project << endl;
	// }
	return move(ret);
}

Matrix TransFormer::cal_homo_transform2(const vector<int>& matches) const {
	int n = matches.size();
	m_assert(n * 2 >= HOMO_FREEDOM);

	Matrix m(9, 2 * n);
	REP(i, n) {
		const Vec2D &m0 = f1[match.data[matches[i]].x].real_coor,
					&m1 = f2[match.data[matches[i]].y].real_coor;
		m.get(i * 2, 0) = m1.x;
		m.get(i * 2, 1) = m1.y;
		m.get(i * 2, 2) = 1;
		m.get(i * 2, 3) = m.get(i * 2, 4) = m.get(i * 2, 5) = 0;
		m.get(i * 2, 6) = -m1.x * m0.x;
		m.get(i * 2, 7) = -m1.y * m0.x;
		m.get(i * 2, 8) = -m0.x;

		m.get(i * 2 + 1, 0) = m.get(i * 2 + 1, 1) = m.get(i * 2 + 1, 2) = 0;
		m.get(i * 2 + 1, 3) = m1.x;
		m.get(i * 2 + 1, 4) = m1.y;
		m.get(i * 2 + 1, 5) = 1;
		m.get(i * 2 + 1, 6) = -m1.x * m0.y;
		m.get(i * 2 + 1, 7) = -m1.y * m0.y;
		m.get(i * 2 + 1, 8) = -m0.y;
	}
	Matrix u(2 * n, 2 * n), v(9, 9), s(9, 2 * n);
	m.SVD(u, s, v);
	Matrix bestcol(1, 9);
	real_t mineigen = numeric_limits<real_t>::max();
	REP(i, 9) {
		Matrix col = v.col(i);
		real_t mod = m.prod(col).sqrsum();
		if (update_min(mineigen, mod))
			bestcol = col;
	}

	Matrix ret(3, 3);
	REP(i, 9) ret.get(i / 3, i % 3) = bestcol.get(i, 0);
	// check
	// for (auto &i : matches) {
	// 	Vec2D project = cal_project(ret, i.second);
	// 	cout << i.first << " == ?" << project << endl;
	// }
	return move(ret);
}


extern bool TEMPDEBUG;

Vec2D TransFormer::cal_project(const Matrix & trans, const Vec2D & old) {
	Matrix m(1, 3);
	m.get(0, 0) = old.x, m.get(1, 0) = old.y, m.get(2, 0) = 1;
	Matrix res = trans.prod(m);
	m_assert(res.h == 3);
	real_t denom = res.get(2, 0);
	// if (fabs(denom) < 1e-2) denom = 1;		// XXX wtf
	Vec2D ret(res.get(0, 0) / denom, res.get(1, 0) / denom);
	return ret;
}

int TransFormer::cal_inliers(const Matrix & trans) const {
	int cnt = 0;
	for (auto & pair : match.data) {
		Vec2D project = TransFormer::cal_project(trans, f2[pair.y].real_coor);
		const Vec2D& fcoor = f1[pair.x].real_coor;
		real_t dist = (project - Vec2D(fcoor.x, fcoor.y)).sqr();
		if (dist < sqr(RANSAC_INLIER_THRES))
			cnt ++;
	}
	return cnt;
}

vector<int> TransFormer::get_inliers(const Matrix & trans) const {
	vector<int> ret;
	int n = match.size();
	REP(i, n) {
		auto &pair = match.data[i];
		Vec2D project = TransFormer::cal_project(trans, f2[pair.y].real_coor);
		const Vec2D& fcoor = f1[pair.x].real_coor;
		real_t dist = (project - Vec2D(fcoor.x, fcoor.y)).sqr();
		if (dist < sqr(RANSAC_INLIER_THRES))
			ret.push_back(i);
	}
	return move(ret);
}

real_t TransFormer::get_focal_from_matrix(const Matrix& m) {
	real_t f2;
	real_t p1 = sqr(m.get(0, 0)) + sqr(m.get(0, 1)) - sqr(m.get(1, 0)) - sqr(m.get(1, 1));
	if (fabs(p1) > EPS)  {
		f2 = (sqr(m.get(1, 2)) - sqr(m.get(0, 2))) / (p1);
	} else {
		p1 = m.get(0, 0) * m.get(1, 0) + m.get(0, 1) * m.get(1, 1);
		if (fabs(p1) > EPS)
			f2 = -(m.get(0, 2) * m.get(1, 2)) / p1;
		else {
			return Vec(m.get(0, 0), m.get(0, 1), m.get(0, 2)).dot(Vec(m.get(2, 0), m.get(2, 1), m.get(2, 2)));
		}
	}
	return sqrt(fabs(f2));
}
