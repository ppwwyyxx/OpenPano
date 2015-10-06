// File: matcher.hh
// Date: Fri May 03 15:45:31 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <memory>
#include "feature/feature.hh"

// TODO. keep actual coordinate in two images
class MatchData {
	public:
		// each pair contains two idx of each match
		std::vector<std::pair<int, int>> data;

		int size() const { return data.size(); }

		void reverse() {
			for (auto& i : data)
				i = std::make_pair(i.second, i.first);
		}
};

class FeatureMatcher {
	protected:
		const std::vector<Descriptor> &feat1, &feat2;
	public:
		FeatureMatcher(const std::vector<Descriptor>& f1, const std::vector<Descriptor>& f2):
			feat1(f1), feat2(f2) { }

		MatchData match() const;


};
