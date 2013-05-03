// File: matcher.cc
// Date: Fri May 03 16:47:18 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <limits>
#include "matcher.hh"
using namespace std;

void MatchData::add(const Coor &t)
{ data.push_back(t); }

MatchData Matcher::match() const {
	MatchData ret;
	int l1 = feat1.size(),
		l2 = feat2.size();

#pragma omp parallel for schedule(dynamic)
	REP(k, l1) {
		const Feature& i = feat1[k];
		int min = numeric_limits<int>::max(),
			minn = min;
		int minkk = 0;
		REP(kk, l2) {
			int dist = cal_dist(i, feat2[kk]);
			if (dist < min) {
				minn = min;
				min = dist;
				minkk = kk;
			} else {
				update_min(minn, dist);
			}
		}
		if (min > sqr(MATCH_REJECT_NEXT_RATIO) * minn) //|| min > 200)
			continue;

#pragma omp critical
		ret.add(Coor(k, minkk));
	}
	return move(ret);
}

int Matcher::cal_dist(const Feature& x, const Feature& y) const {
	int ans = 0;
	for (int i = 0; i < DESC_LEN; i ++)
		ans += sqr(x.descriptor[i] - y.descriptor[i]);		// use euclidean
	return ans;
}
