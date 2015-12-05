//File: camera_estimator.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "camera_estimator.hh"
#include <queue>

#include "lib/debugutils.hh"
#include "lib/timer.hh"
#include "lib/utils.hh"
#include "lib/config.hh"
#include "camera.hh"
#include "bundle_adjuster.hh"
#include "incremental_bundle_adjuster.hh"

using namespace std;
using namespace config;

namespace stitch {

vector<Camera> CameraEstimator::estimate() {
	{ // assign an initial focal length
		double focal = Camera::estimate_focal(matches);
		if (focal > 0) {
			for (auto& c : cameras)
				c.focal = focal;
			print_debug("Estimated focal: %lf\n", focal);
		} else {
			print_debug("Cannot estimate focal. Will use a naive one.");
			REP(i, n) // hack focal
				cameras[i].focal = (shapes[i].w + shapes[i].h) * 0.5;
		}
	}

	auto graph = max_spanning_tree();
	propagate_rotation(graph);

	{
		GuardedTimer tm("IncrementalBundleAdjuster");
		IncrementalBundleAdjuster iba(shapes, cameras);
		REP(i, n) {
			REPL(j, i+1, n) {
				auto& m = matches[j][i];
				if (m.match.size())
					iba.add_match(i, j, m);
			}
			// TODO optimize after every k images
			if (MULTIPASS_BA)	// optimize after adding every image
				iba.optimize();
		}
		if (not MULTIPASS_BA)
			iba.optimize();
	}

	/*
	 *BundleAdjuster ba(shapes, matches);	// old BA
	 *ba.estimate(cameras);
	 */

	if (STRAIGHTEN) Camera::straighten(cameras);
	return cameras;
}

vector<vector<int>> CameraEstimator::max_spanning_tree() {
	struct Edge {
		int v1, v2;
		float weight;
		bool have(int v) { return v1 == v || v2 == v; }
		Edge(int a, int b, float v):v1(a), v2(b), weight(v) {}
		bool operator < (const Edge& r) const
		{ return weight > r.weight;	}
	};

	vector<vector<int>> graph(n);

	vector<Edge> edges;
	REP(i, n) REPL(j, i+1, n) {
		auto& m = matches[i][j];
		if (m.confidence <= 0) continue;
		edges.emplace_back(i, j, m.confidence);
	}
	sort(edges.begin(), edges.end());		// large weight to small weight
	vector<bool> in_tree(n, false);
	int edge_cnt = 0;
	in_tree[edges.front().v1] = true;
	while (true) {
		int old_edge_cnt = edge_cnt;
		auto itr = begin(edges);
		for (; itr != edges.end(); ++itr) {
			Edge& e = *itr;
			if (in_tree[e.v1] && in_tree[e.v2]) {
				edges.erase(itr);
				break;
			}
			if (not in_tree[e.v1] && not in_tree[e.v2])
				continue;
			in_tree[e.v1] = in_tree[e.v2] = true;
			graph[e.v1].push_back(e.v2);
			graph[e.v2].push_back(e.v1);
			print_debug("MST: Best edge from %d to %d\n", e.v1, e.v2);
			edges.erase(itr);
			edge_cnt ++;
			break;
		}
		if (edge_cnt == n - 1) // tree is full
			break;
		if (edge_cnt == old_edge_cnt && itr == edges.end())
			// no edge to add
			break;
	}
	if (edge_cnt != n - 1)
		error_exit(ssprintf(
					"Found a tree of size %d!=%d, images are not connected well!",
					edge_cnt, n - 1));
	return graph;
}

void CameraEstimator::propagate_rotation(const Graph& graph) {
	int start = 0;	// TODO select root of tree (best confidence)

	queue<int> q; q.push(start);
	vector<bool> vst(graph.size(), false);		// in queue
	vst[start] = true;
	while (q.size()) {
		int now = q.front(); q.pop();
		for (int next: graph[now]) {
			if (vst[next]) continue;
			vst[next] = true;
			// from now to next
			auto Kfrom = cameras[now].K();
			auto Kto = cameras[next].K();
			auto Hinv = matches[now][next].homo;	// from next to now
			auto Mat = Kfrom.inverse() * Hinv * Kto;
			cameras[next].R = (cameras[now].Rinv() * Mat).transpose();
			// this is camera extrincis R, i.e. going from identity to this image
			q.push(next);
		}
	}
	REP(i, n) {
		cameras[i].ppx = shapes[i].halfw();
		cameras[i].ppy = shapes[i].halfh();
	}
}
}
