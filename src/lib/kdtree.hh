//File: kdtree.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include <queue>
#include <functional>
#include <limits>
#include "lib/debugutils.hh"

namespace pano {

class KDTree {
	public:
		typedef std::vector<float> Point;

		struct NNResult {
			int idx;
			float sqrdist;
		};

		struct TwoNNResult {
			int idx;
			float sqrdist, sqrdist2;
		};

		KDTree(const std::vector<const Point*> & points);

		NNResult nearest_neighbor(const Point& p) const {
			return nn_in_node(p, root, std::numeric_limits<float>::max());
		}

		TwoNNResult two_nearest_neighbor(const Point& p) const {
			return two_nn_in_node(p, root, std::numeric_limits<float>::max());
		}

	private:
		// internal structures:
		struct Node;
		typedef std::pair<int, const Point*> PointInBuild;

		struct Split {
			int axis;
			float min, range;

			Split(int a, float m, float r):
				axis(a), min(m), range(r) {}

			inline bool operator < (const Split& d) const
			{ return range < d.range; }

			inline float max() const { return min + range; }
		};
		typedef std::priority_queue<Split> DimStats;

		// members:
		Node* root;
		int D;		// dimension

		// helper functions:
		Node* build(
				std::vector<PointInBuild> & points,
				DimStats&, int depth);

		DimStats build_dim_stat(const std::vector<const Point*>& pts);

		// thres: no need to return a point further than thres
		NNResult nn_in_node(const Point& p, Node*, float) const;
		TwoNNResult two_nn_in_node(const Point& p, Node*, float) const;
};

}
