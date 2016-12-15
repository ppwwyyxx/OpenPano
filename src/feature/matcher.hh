// File: matcher.hh
// Date: Fri May 03 15:45:31 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <flann/flann.hpp>
#include "feature.hh"
#include "dist.hh"
#include "common/common.hh"

namespace pano {

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

		FeatureMatcher(const FeatureMatcher&) = delete;
		FeatureMatcher& operator = (const FeatureMatcher&) = delete;

		MatchData match() const;
};

class PairWiseMatcher {
	public:
		explicit PairWiseMatcher(
				const std::vector<std::vector<Descriptor>>& feats)
			: D(feats.at(0).at(0).descriptor.size()), feats(feats)
		{ build(); }

		PairWiseMatcher(const PairWiseMatcher&) = delete;
		PairWiseMatcher& operator = (const PairWiseMatcher&) = delete;

		// return pair of <idx in i, idx in j>
		MatchData match(int i, int j) const;

		~PairWiseMatcher() {
			for (auto& p: feature_bufs) delete[] p;
		}

	protected:
		const int D; // feature dimension
		const std::vector<std::vector<Descriptor>> &feats;

		std::vector<flann::Index<pano::L2SSE>> trees;

    // feature_bufs[i] is a buffer of size feats[i].size() * D
		std::vector<float*> feature_bufs;

		void build();
};

}
