// File: matcher.hh
// Date: Fri May 03 15:45:31 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <memory>
#include "feature/feature.hh"

class MatchData {
	public:
		std::vector<Coor> data;
		MatchData(){}

		MatchData(const MatchData& r):
			data(r.data) {}

		MatchData(MatchData&& r):
			data(std::move(r.data)) {}

		void add(const Coor&);

		int size() const { return data.size(); }

		MatchData reverse() const {
			MatchData ret(*this);
			for (auto &i : ret.data)
				i = Coor(i.y, i.x);
			return std::move(ret);
		}
};

class Matcher {
	protected:
		const std::vector<Descriptor> &feat1, &feat2;

		int cal_dist(const Descriptor& x, const Descriptor& y) const; // return the squared value
	public:
		Matcher(const std::vector<Descriptor>& f1, const std::vector<Descriptor>& f2):
			feat1(f1), feat2(f2) { }

		MatchData match() const;


};
