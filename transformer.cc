// File: transformer.cc
// Date: Sun Apr 21 19:35:43 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "transformer.hh"
#include <set>
using namespace std;

Matrix TransFormer::get_transform() {
	int n_match = match.size();
	if (n_match < MATCH_MIN_SIZE)
		m_assert(false);

	int selected[AFFINE_REQUIRED_MATCH];
	vector<pair<Coor, Coor>> inliers;
	inliers.reserve(AFFINE_REQUIRED_MATCH);

	for (int K = RANSAC_ITERATIONS; K --;) {
		for (int i = 0; i < AFFINE_REQUIRED_MATCH; i ++) {
			int random;
			while (1) {
				random = rand() % n_match;

				bool vst = false;
				for (int j = 0; j < i; j ++)
					if (selected[j] == random) {
						vst = true;
						continue;
					}
				if (!vst) break;
			}

			selected[i] = random;
			inliers.push_back(match.data[i]);
		}

		Matrix transform = compute_transform(inliers);

	}

}

Matrix TransFormer::compute_transform(const vector<pair<Coor, Coor>>& matches) const {
	int n = matches.size();
	m_assert(n >= AFFINE_REQUIRED_MATCH);

	Matrix m(2 * 4, 2 * n);		// 8 degree of freedom
	Matrix b(1, 2 * n);
	for (int i = 0; i < n; i ++) {
		const Coor &m0 = matches[i].second,
				   &m1 = matches[i].first;
		m.get(i * 2, 0) = m1.x;
		m.get(i * 2, 1) = m1.y;
		m.get(i * 2, 2) = 1;
		m.get(i * 2, 3) = m.get(i * 2, 4) = m.get(i * 2, 5) = 0;
		m.get(i * 2, 6) = -m1.x * m0.x;
		m.get(i * 2 + 1, 0) = m.get(i * 2 + 1, 1) = m.get(i * 2 + 1, 2) = 0;
		m.get(i * 2 + 1, 3) = m1.x;
		m.get(i * 2 + 1, 4) = m1.y;
		m.get(i * 2 + 1, 5) = 1;
		m.get(i * 2 + 1, 6) = -m1.x * m0.y;
		m.get(i * 2 + 1, 7) = -m1.y * m0.y;

		b.get(i * 2, 0) = m0.x, b.get(i * 2 + 1, 0) = m0.y;
	}


}
