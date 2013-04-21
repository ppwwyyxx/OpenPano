// File: transformer.cc
// Date: Sun Apr 21 16:40:23 2013 +0800
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
	m_assert(matches.size() >= AFFINE_REQUIRED_MATCH);

}
