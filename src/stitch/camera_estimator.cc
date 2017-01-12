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
#include "match_info.hh"
#include "incremental_bundle_adjuster.hh"

using namespace std;
using namespace config;

namespace pano {

CameraEstimator::CameraEstimator(
    std::vector<std::vector<MatchInfo>>& matches,
    const std::vector<Shape2D>& image_shapes) :
  n(matches.size()),
  matches(matches),
  shapes(image_shapes),
  cameras(matches.size())
{ m_assert(matches.size() == shapes.size()); }

CameraEstimator::~CameraEstimator() = default;

void CameraEstimator::estimate_focal() {
  // assign an initial focal length
  double focal = Camera::estimate_focal(matches);
  if (focal > 0) {
    for (auto& c : cameras)
      c.focal = focal;
    print_debug("Estimated focal: %lf\n", focal);
  } else {
    print_debug("Cannot estimate focal. Will use a naive one.\n");
    REP(i, n) // hack focal
      cameras[i].focal = (shapes[i].w + shapes[i].h) * 0.5;
  }
}

vector<Camera> CameraEstimator::estimate() {
  GuardedTimer tm("Estimate Camera");
  estimate_focal();

  IncrementalBundleAdjuster iba(cameras);
  vector<bool> vst(n, false);
  traverse(
      [&](int node) {
        // set the starting point to identity
        cameras[node].R = Homography::I();
        cameras[node].ppx = cameras[node].ppy = 0;
      },
      [&](int now, int next) {
        print_debug("Best edge from %d to %d\n", now, next);
        auto Kfrom = cameras[now].K();

        // initialize camera[next]:
        auto Kto = cameras[next].K();
        auto Hinv = matches[now][next].homo;	// from next to now
        auto Mat = Kfrom.inverse() * Hinv * Kto;
        // this is camera extrincis R, i.e. going from identity to this image
        cameras[next].R = (cameras[now].Rinv() * Mat).transpose();
        cameras[next].ppx = cameras[next].ppy = 0;
        // initialize by the last camera. It shouldn't fail but it did,
        // may be deficiency in BA, or because of ignoring large error for now
        //cameras[next] = cameras[now];

        if (MULTIPASS_BA > 0) {
          // add next to BA
          vst[now] = vst[next] = true;
          REP(i, n) if (vst[i] && i != next) {
            const auto& m = matches[next][i];
            if (m.match.size() && m.confidence > 0) {
              iba.add_match(i, next, m);
              if (MULTIPASS_BA == 2) {
                print_debug("MULTIPASS_BA: %d -> %d\n", next, i);
                iba.optimize();
              }
            }
          }
          if (MULTIPASS_BA == 1)
            iba.optimize();
        }
      });

  if (MULTIPASS_BA == 0) {		// optimize everything together
    REPL(i, 1, n) REP(j, i) {
      auto& m = matches[j][i];
      if (m.match.size() && m.confidence > 0)
        iba.add_match(i, j, m);
    }
    iba.optimize();
  }

  if (STRAIGHTEN) Camera::straighten(cameras);
  return cameras;
}

void CameraEstimator::traverse(
    function<void(int)> callback_init_node,
    function<void(int, int)> callback_edge) {
  struct Edge {
    int v1, v2;
    float weight;
    Edge(int a, int b, float v):v1(a), v2(b), weight(v) {}
    bool operator < (const Edge& r) const { return weight < r.weight;	}
  };
  // choose a starting point
  Edge best_edge{-1, -1, 0};
  REP(i, n) REPL(j, i+1, n) {
    auto& m = matches[i][j];
    if (m.confidence > best_edge.weight)
      best_edge = Edge{i, j, m.confidence};
  }
  if (best_edge.v1 == -1)
    error_exit("No connected images are found!");
  callback_init_node(best_edge.v1);

  priority_queue<Edge> q;
  vector<bool> vst(n, false);

  auto enqueue_edges_from = [&](int from) {
    REP(i, n) if (i != from && !vst[i]) {
      auto& m = matches[from][i];
      if (m.confidence > 0)
        q.emplace(from, i, m.confidence);
    }
  };

  vst[best_edge.v1] = true;
  enqueue_edges_from(best_edge.v1);
  int cnt = 1;
  while (q.size()) {
    do {
      best_edge = q.top();
      q.pop();
    } while (q.size() && vst[best_edge.v2]);
    if (vst[best_edge.v2])	// the queue is exhausted
      break;
    vst[best_edge.v2] = true;
    cnt ++;
    callback_edge(best_edge.v1, best_edge.v2);
    enqueue_edges_from(best_edge.v2);
  }
  if (cnt != n) {
    string unconnected;
    REP(i, n) if (not vst[i])
      unconnected += to_string(i) + " ";
    error_exit(ssprintf(
          "Found a tree of size %d!=%d, image %s are not connected well!",
          cnt, n, unconnected.c_str()));
  }
}
}
