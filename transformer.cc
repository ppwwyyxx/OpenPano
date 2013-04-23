// File: transformer.cc
// Date: Tue Apr 23 18:46:58 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "transformer.hh"
#include <set>
using namespace std;

Matrix TransFormer::get_transform() {		// second -> first
	int REQUIRED = (USE_HOMO ? HOMO_FREEDOM / 2 : AFFINE_FREEDOM / 2);
	int n_match = match.size();
	if (n_match < MATCH_MIN_SIZE)
		m_assert(false);

	vector<pair<Vec2D, Vec2D>> fit;
	set<int> selected;

	int maxinlierscnt = 0;
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
			fit.push_back(match.data[random]);
		}
		Matrix transform = cal_transform(fit);
		int inlier = cal_inliers(transform);
		// int inlier = get_inliers(transform).size();
		if (update_max(maxinlierscnt, inlier)) {
			best_transform = move(transform);
		}
	}
	m_assert(maxinlierscnt > 0);
	cout << "max num of inlier: " << maxinlierscnt << endl;

	auto inliers = get_inliers(best_transform);
	best_transform = cal_transform(inliers);
	inliers = get_inliers(best_transform);
	best_transform = cal_transform(inliers);
	cout << "final num of inlier: " << inliers.size() << endl;
	return move(best_transform);
}

Matrix TransFormer::cal_transform(const vector<pair<Vec2D, Vec2D>>& matches) const {
	if (USE_HOMO)
		return move(cal_homo_transform(matches));
	else
		return move(cal_affine_transform(matches));
}

Matrix TransFormer::cal_affine_transform(const vector<pair<Vec2D, Vec2D>>& matches) const {
	int n = matches.size();
	m_assert(n * 2 >= AFFINE_FREEDOM);

	Matrix m(AFFINE_FREEDOM, 2 * n);
	Matrix b(1, 2 * n);
	REP(i, n) {
		const Vec2D &m0 = matches[i].first, &m1 = matches[i].second;
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
	Matrix res(0, 0);
	if (!m.solve_overdetermined(res, b)) { cout << "solve failed" << endl; return move(res); }
	Matrix ret(3, 3);
	REP(i, AFFINE_FREEDOM) ret.get(i / 3, i % 3) = res.get(i, 0);
	ret.get(2, 2) = 1;
	return move(ret);
}

// second -> first
Matrix TransFormer::cal_homo_transform(const vector<pair<Vec2D, Vec2D>>& matches) const {
	int n = matches.size();
	m_assert(n * 2 >= HOMO_FREEDOM);

	Matrix m(HOMO_FREEDOM, 2 * n);
	Matrix b(1, 2 * n);
	REP(i, n) {
		const Vec2D &m0 = matches[i].first, &m1 = matches[i].second;
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
	Matrix res(0, 0);
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

Vec2D TransFormer::cal_project(const Matrix & trans, const Vec2D & old) {
	Matrix m(1, 3);
	m.get(0, 0) = old.x, m.get(1, 0) = old.y, m.get(2, 0) = 1;
	Matrix res = trans.prod(m);
	m_assert(res.h == 3);
	real_t denom = res.get(2, 0);
	Vec2D ret(res.get(0, 0) / denom, res.get(1, 0) / denom);
	return ret;
}

int TransFormer::cal_inliers(const Matrix & trans) const {
	int cnt = 0;
	for (auto & pair : match.data) {
		Vec2D project = TransFormer::cal_project(trans, pair.second);
		real_t dist = (project - Vec2D(pair.first.x, pair.first.y)).sqr();
		if (dist < sqr(RANSAC_INLIER_THRES))
			cnt ++;
	}
	return cnt;
}

vector<pair<Vec2D, Vec2D>> TransFormer::get_inliers(const Matrix & trans) const {
	vector<pair<Vec2D, Vec2D>> ret;
	for (auto & pair : match.data) {
		Vec2D project = TransFormer::cal_project(trans, pair.second);
		real_t dist = (project - Vec2D(pair.first.x, pair.first.y)).sqr();
		if (dist < sqr(RANSAC_INLIER_THRES))
			ret.push_back(pair);
	}
	return move(ret);
}
