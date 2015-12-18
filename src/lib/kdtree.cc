//File: kdtree.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>
//
#include "kdtree.hh"

#include <limits>
#include <algorithm>

#include "feature/dist.hh"
#include "lib/utils.hh"
#include "lib/debugutils.hh"
using namespace std;

namespace pano {

struct KDTree::Node {
	Node* child[2];

	union {
		struct {				 // non-leaf
			float split;
			int axis;
		};
		vector<PointInBuild> pts; // leaf. holding original indices
	};

	Node() {}

	~Node() {
		delete child[0]; delete child[1];
		if (leaf()) pts.~vector<PointInBuild>();
	}

	bool leaf() const
	{ return !child[0] /*&& !child[1]*/; }
};


KDTree::KDTree(
		const std::vector<const Point*> & points) {
	m_assert(points.size());
	D = points[0]->size();

	vector<PointInBuild> ptsptr;
	ptsptr.reserve(points.size());
	REP(i, points.size())
		 ptsptr.emplace_back(i, points[i]);

	DimStats stats = build_dim_stat(points);
	root = build(ptsptr, stats, 0);
}

KDTree::DimStats KDTree::build_dim_stat(
		const std::vector<const Point*>& pts) {
	DimStats ret;
	vector<float> max(D, 0);
	vector<float> min(D, numeric_limits<float>::max());
	for (auto& p : pts) {
		REP(i, D) {
			update_max(max[i], (*p)[i]);
			update_min(min[i], (*p)[i]);
		}
	}
	REP(i, D)
		ret.emplace(i, min[i], max[i] - min[i]);
	return ret;
}

KDTree::Node* KDTree::build(
		vector<PointInBuild> & points,
		DimStats& stat, int depth) {
	Split sp = stat.top(); stat.pop();
	int split_axis = sp.axis;
	//split_axis = random() % D;
	//print_debug("Depth %d, Split at %d, mid = %f, pts=%lu\n", depth, split_axis, sp.min + sp.range * 0.5, points.size());
	Node* ret = new Node();

	// TODO parameter
	bool STOP = (sp.range < 10 || points.size() < 20 || depth > 5);
	if (STOP) {
		ret->child[0] = ret->child[1] = nullptr;
		new (&ret->pts) vector<int>;
		ret->pts = move(points);
		return ret;
	}

	// recursively build
	ret->axis = split_axis;

	// find median to split
	/*
	 *vector<float> vals;
	 *for (auto& p : points) vals.emplace_back((*p.second)[split_axis]);
	 *nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
	 *float split_val = vals[vals.size() / 2];
	 */
	float split_val = sp.min + sp.range * 0.5;
	ret->split = split_val;

	vector<PointInBuild> right;
	vector<PointInBuild> left;
	float minl = sp.max(), maxl = 0, minr = sp.max(), maxr = 0;
	for (auto& p : points) {
		float v = (*p.second)[split_axis];
		if (v > split_val) {
			right.emplace_back(p);
			update_max(maxr, v);
			update_min(minr, v);
		} else {
			left.emplace_back(p);
			update_max(maxl, v);
			update_min(minl, v);
		}
	}

	DimStats rstat = stat;
	rstat.emplace(split_axis, minr, maxr - minr);
	DimStats lstat = move(stat);
	lstat.emplace(split_axis, minl, maxl - minl);

	ret->child[0] = build(left, lstat, depth + 1);
	ret->child[1] = build(right, rstat, depth + 1);
	return ret;
}

KDTree::NNResult KDTree::nn_in_node(const Point& p, Node* n, float thres) const {
	if (n->leaf()) {
		float min_d = thres;
		int min_i = -1;
		for (auto& lp: n->pts) {
			float v = euclidean_sqr(
					lp.second->data(), p.data(), D, min_d);
			if (update_min(min_d, v))
				min_i = lp.first;
		}
		return NNResult{min_i, min_d};
	}

	float split = n->split,
				dist = p[n->axis] - split;
	int left = dist <= 0;
	dist = dist * dist; // + (D-1) * 100;	// dist to plane

	// if left, then 0 first
	NNResult ret = nn_in_node(p, n->child[1-left], thres);
	if (ret.idx == -1) {
		return nn_in_node(p, n->child[left], thres);
	} else {
		m_assert(ret.sqrdist < thres);
		if (ret.sqrdist < dist)		// must be this one!
			return ret;

		// try another
		NNResult ret2 = nn_in_node(p, n->child[left], ret.sqrdist);
		if (ret2.idx == -1 || ret.sqrdist <= ret2.sqrdist)
			return ret;
		return ret2;
	}

}

KDTree::TwoNNResult KDTree::two_nn_in_node(const Point& p, Node* n, float thres) const {
	if (n->leaf()) {
		float min_d = thres, min_d2 = thres;
		int min_i = -1;
		for (auto& lp: n->pts) {		// leaf point
			float v = euclidean_sqr(
					lp.second->data(), p.data(), D, min_d2);
			if (v < min_d)
				min_d2 = min_d, min_d = v, min_i = lp.first;
			else
				update_min(min_d2, v);
		}
		return TwoNNResult{min_i, min_d, min_d2};
	}

	float split = n->split,
				dist = p[n->axis] - split;
	int first = dist > 0;
	dist = dist * dist;	// dist to plane

	// if left, then 0 first
	auto ret = two_nn_in_node(p, n->child[first], thres);
	if (ret.idx == -1) {
		return two_nn_in_node(p, n->child[1-first], thres);
	} else {
		m_assert(ret.sqrdist < thres);
		if (ret.sqrdist2 < dist)		// must be this one!
			return ret;
		if (ret.sqrdist < dist) {		// second largest might be in another part
			auto ret2 = nn_in_node(p, n->child[1-first], ret.sqrdist2);
			update_min(ret.sqrdist2, ret2.sqrdist);
			return ret;
		}

		// both could be in another part
		auto ret2 = two_nn_in_node(p, n->child[1-first], ret.sqrdist2);

		if (ret2.idx == -1)
			return ret;
		if (ret2.sqrdist < ret.sqrdist) {
			ret.sqrdist2 = ret.sqrdist;
			ret.sqrdist = ret2.sqrdist;
			ret.idx = ret2.idx;
			update_min(ret.sqrdist2, ret2.sqrdist2);
		} else
			update_min(ret.sqrdist2, ret2.sqrdist);
		return ret;
	}
}

}
