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
		print_debug("Min: %lf, %lf\n", min, next_min);

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
	/*
	 *match(0, 1);
	 *exit(0);
	 *trees[0].two_nearest_neighbor(feats[0][0].descriptor);
	 */
	/*
	 *for (auto& s : feats[0])
	 *  for (auto& p : feats[1])
	 *    s.euclidean_sqr(p, 1);
	 *exit(0);
	 */
}

MatchData PairWiseEuclideanMatcher::match(int i, int j) const {
	TotalTimer tm("pairwise match match");
	static const float REJECT_RATIO_SQR = MATCH_REJECT_NEXT_RATIO * MATCH_REJECT_NEXT_RATIO;
	MatchData ret;
	auto source = feats[i],
			 target = feats[j];
	auto t = trees[j];
#pragma omp parallel for schedule(dynamic)
	REP(i, source.size()) {
		auto& s = source[i];
		auto res = t.two_nearest_neighbor(s.descriptor);

		// check kdtree impl correctness
		/*
		 *float mind = numeric_limits<float>::max(), mind2 = mind;
		 *int mini = -1;
		 *REP(k, target.size()) {
		 *  float d = s.euclidean_sqr(target[k], mind2);
		 *  if (d < mind) {
		 *    mind2 = mind;
		 *    mind = d;
		 *    mini = k;
		 *  } else
		 *    update_min(mind2, d);
		 *}
		 *PP(mind2); PP(res.sqrdist2);
		 *PP(mind); PP(res.sqrdist);
		 *PP(mini); PP(res.idx);
		 *m_assert(mind2 == res.sqrdist2);
		 *m_assert(mind == res.sqrdist);
		 *m_assert(mini == res.idx);
		 */

		if (res.sqrdist > REJECT_RATIO_SQR * res.sqrdist2)
			continue;
#pragma omp critical
		ret.data.emplace_back(i, res.idx);
	}
	return ret;
}

}
