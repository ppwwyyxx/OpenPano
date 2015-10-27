// File: matcher.cc
// Date: Fri May 03 17:02:09 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <limits>
#include "matcher.hh"
#include "lib/timer.hh"
using namespace std;


namespace feature {

MatchData FeatureMatcher::match() const {
	static const float REJECT_RATIO_SQR = MATCH_REJECT_NEXT_RATIO * MATCH_REJECT_NEXT_RATIO;
	TotalTimer tm("matcher");

	size_t l1 = feat1.size(), l2 = feat2.size();
	// loop over the smaller one to speed up
	bool rev = l1 > l2;
	const vector<Descriptor> *pf1, *pf2;
	if (rev) {
		swap(l1, l2);
		pf1 = &feat2, pf2 = &feat1;
	} else {
		pf1 = &feat1, pf2 = &feat2;
	}

	MatchData ret;

#pragma omp parallel for schedule(dynamic)
	REP(k, l1) {
		const Descriptor& i = (*pf1)[k];
		int min_idx = -1;

		// TODO use knn for sift
		float min = numeric_limits<float>::max(),
					next_min = min;
		REP(kk, l2) {
			float dist = i.euclidean_sqr((*pf2)[kk], next_min);
			if (dist < min) {
				next_min = min;
				min = dist;
				min_idx = kk;
			} else {
				update_min(next_min, dist);
			}
		}
		if (min > REJECT_RATIO_SQR * next_min)
			continue;

		m_assert(min_idx != -1);
#pragma omp critical
		ret.data.emplace_back(k, min_idx);
	}
	if (rev)
		ret.reverse();
	return ret;
}

void PairWiseEuclideanMatcher::build() {
	TotalTimer tm("BuildTrees");
	for (auto& feat: feats)	{
		vector<const vector<float>*> pts;
		pts.reserve(feat.size());
		for (auto& desc: feat)
			pts.emplace_back(&desc.descriptor);
		trees.emplace_back(pts);
	}

	// test:
	/*
	 *  auto source = feats[0];
	 *  auto target = feats[1];
	 *
	 *  for (auto& s : source) {
	 *    trees[1].set_dist_func([&](int k, float th) {
	 *        return s.euclidean_sqr(target[k], th);
	 *        });
	 *    auto res = trees[1].two_nearest_neighbor(s.descriptor);
	 *    PP(res.idx); PP(res.sqrdist); PP(res.sqrdist2);
	 *
	 *    float mind = 1e9, mind2 = 1e9; int mini = -1;
	 *    REP(k, target.size()) {
	 *      float d = s.euclidean_sqr(target[k], mind2);
	 *      if (d < mind) {
	 *        mind2 = mind;
	 *        mind = d;
	 *        mini = k;
	 *      } else
	 *        update_min(mind2, d);
	 *    }
	 *    PP(mini); PP(mind); PP(mind2);
	 *    m_assert(mind2 == res.sqrdist2);
	 *
	 *    m_assert(mind == res.sqrdist);
	 *    m_assert(mini == res.idx);
	 *  }
	 *  exit(0);
	 */
}

MatchData PairWiseEuclideanMatcher::match(int i, int j) const {
	TotalTimer tm("pairwise match match");
	static const float REJECT_RATIO_SQR = MATCH_REJECT_NEXT_RATIO * MATCH_REJECT_NEXT_RATIO;
	MatchData ret;
	auto source = feats[i],
			 target = feats[j];
	auto t = trees[j];
	REP(i, source.size()) {
		auto& s = source[i];
		t.set_dist_func([&](int k, float th) {
				return s.euclidean_sqr(target[k], th);
				});
		auto res = t.two_nearest_neighbor(s.descriptor);
		if (res.sqrdist > REJECT_RATIO_SQR * res.sqrdist2)
			continue;
		ret.data.emplace_back(i, res.idx);
	}
	return ret;
}

}
