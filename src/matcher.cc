// File: matcher.cc
// Date: Fri May 03 17:02:09 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <limits>
#include "matcher.hh"
#include "lib/timer.hh"
using namespace std;

MatchData Matcher::match() const {
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
		float min = numeric_limits<float>::max(),
					next_min = min;
		int min_idx = 0;
		REP(kk, l2) {
			float dist = i.euclidean_sqr((*pf2)[kk], next_min);
			if (dist < 0) continue;
			if (dist < min) {
				next_min = min;
				min = dist;
				min_idx = kk;
			} else {
				update_min(next_min, dist);
			}
		}
		if (min > std::min(
					REJECT_RATIO_SQR * next_min, 40000.0f))
			continue;

#pragma omp critical
		ret.data.emplace_back(k, min_idx);
	}
	if (rev)
		return ret.reverse();
	else
		return ret;
}
