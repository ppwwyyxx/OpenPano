// File: matcher.cc
// Date: Fri May 03 17:02:09 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <limits>
#include <flann/flann.hpp>
#include "matcher.hh"
#include "lib/timer.hh"
#include "feature.hh"
using namespace std;
using namespace config;

#ifdef _MSC_VER
// necessary to define here since flann doesn't provide serialization for size_t as unsigned long long
namespace flann {
  namespace serialization {
    //BASIC_TYPE_SERIALIZER(size_t);
  }
}
#endif

namespace pano {

MatchData FeatureMatcher::match() const {
	static const float REJECT_RATIO_SQR = MATCH_REJECT_NEXT_RATIO * MATCH_REJECT_NEXT_RATIO;
	TotalTimer tm("matcher");

	int l1 = feat1.size(), l2 = feat2.size();
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
		const Descriptor& dsc1 = (*pf1)[k];
		int min_idx = -1;

		float min = numeric_limits<float>::max(),
					next_min = min;
    // find top-2 NN of dsc1 from feat2
		REP(kk, l2) {
			float dist = dsc1.euclidean_sqr((*pf2)[kk], next_min);
			if (dist < min) {
				next_min = min;
				min = dist;
				min_idx = kk;
			} else {
				update_min(next_min, dist);
			}
		}
    /// bidirectional rejection:
    // fix k, see if min_idx is distinctive among feat2
		if (min > REJECT_RATIO_SQR * next_min)
			continue;

    // fix min_idx, see if k is distinctive among feat1
    // next_min remains a large-enough number here, don't need to re-initialize
    const Descriptor& dsc2 = (*pf2)[min_idx];
    REP(kk, l1) if (kk != k) {
      float dist = dsc2.euclidean_sqr((*pf1)[kk], next_min);
      update_min(next_min, dist);
    }
    if (min > REJECT_RATIO_SQR * next_min)
      continue;

#pragma omp critical
    ret.data.emplace_back(k, min_idx);
  }
  if (rev)
    ret.reverse();
  return ret;
}

void PairWiseMatcher::build() {
  GuardedTimer tm("BuildTrees");
  for (auto& feat: feats)	{
    float* buf = new float[feat.size() * D];
    feature_bufs.emplace_back(buf);
    REP(i, feat.size()) {
      float* row = buf + D * i;
      memcpy(row, feat[i].descriptor.data(), D * sizeof(float));
    }
    flann::Matrix<float> points(buf, feat.size(), D);
    trees.emplace_back(points, flann::KDTreeIndexParams(FLANN_NR_KDTREE));	// TODO param
  }
#pragma omp parallel for schedule(dynamic)
  REP(i, (int)trees.size())
    trees[i].buildIndex();
}

MatchData PairWiseMatcher::match(int id1, int id2) const {
  static const float REJECT_RATIO_SQR = MATCH_REJECT_NEXT_RATIO * MATCH_REJECT_NEXT_RATIO;
	// loop over the smaller one to speed up
  bool rev = feats[id1].size() > feats[id2].size();
  if (rev) swap(id1, id2);

  auto source = feats[id1], target = feats[id2];

  const flann::Matrix<float> query(feature_bufs[id1], source.size(), D);
  flann::Matrix<int> indices(new int[source.size() * 2], source.size(), 2);
  flann::Matrix<float> dists(new float[source.size() * 2], source.size(), 2);
  trees[id2].knnSearch(query, indices, dists, 2, flann::SearchParams(128));	// TODO param

  MatchData ret;
  float* buf = new float[D];
  flann::Matrix<int> indices_inv(new int[2], 1, 2);
  flann::Matrix<float> dists_inv(new float[2], 1, 2);
  REP(i, source.size()) {
    int mini = indices[i][0];
    float mind = dists[i][0], mind2 = dists[i][1];
    // 1-way rejection:
    if (mind > REJECT_RATIO_SQR * mind2)
      continue;

    // bidirectional rejection:
    //memcpy(buf, target[mini].descriptor.data(), D * sizeof(float));
    flann::Matrix<float> query(target[mini].descriptor.data(), 1, D);
    trees[id1].knnSearch(query, indices_inv, dists_inv, 2, flann::SearchParams(128));
    size_t bidirectional_mini = indices_inv[0][0];
    mind2 = dists_inv[0][1];
    if (bidirectional_mini != i)
      continue;
    if (mind > REJECT_RATIO_SQR * mind2)
      continue;

    ret.data.emplace_back(i, mini);
  }
  if (rev)
    ret.reverse();
  delete[] indices.ptr();
  delete[] dists.ptr();
  delete[] indices_inv.ptr();
  delete[] dists_inv.ptr();
  delete[] buf;
  return ret;
}

}
