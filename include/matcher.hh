// File: matcher.hh
// Date: Sat Apr 20 14:54:03 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <utility>
#include "feature.hh"

class Matcher {
	protected:
		const std::vector<Feature> &feat1, &feat2;

		real_t cal_dist(const Feature& x, const Feature& y) const;
	public:
		Matcher(const std::vector<Feature>& f1, const std::vector<Feature>& f2):
			feat1(f1), feat2(f2) {

		}

		std::vector<std::pair<Coor, Coor>> match() const;


};
