// File: matcher.cc
// Date: Sat Apr 20 15:02:41 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <limits>
#include "matcher.hh"
using namespace std;

void MatchData::add(Coor x, Coor y) {
	data.push_back({x, y});
}

MatchData Matcher::match() const {
	MatchData ret;
	int l1 = feat1.size();

#pragma omp parallel for schedule(dynamic)
	for (int k = 0; k < l1; k ++) {
		const Feature& i = feat1[k];
		int min = numeric_limits<int>::max();
		Coor mincoor;
		for (auto &j : feat2) {
			real_t dist = cal_dist(i, j);
			if (dist < min) {
				min = dist;
				mincoor = j.real_coor;
			}
		}
		if (min > 100) continue;

#pragma omp critical
		ret.add(i.real_coor, mincoor);
	}
	cout << "match: " << ret.size() << endl;
	return move(ret);
}

real_t Matcher::cal_dist(const Feature& x, const Feature& y) const {
	real_t ans = 0;
	for (int i = 0; i < DESC_LEN; i ++)
		ans += sqr(x.descriptor[i] - y.descriptor[i]);		// use euclidean
	return sqrt(ans);
}
