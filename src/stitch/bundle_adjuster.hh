//File: bundle_adjuster.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include "lib/mat.h"
#include "lib/common.hh"
#include <vector>
#include <set>
#include <Eigen/Dense>


namespace stitch {
class Camera;
struct MatchInfo;
struct Shape2D;

class BundleAdjuster {
	public:
		BundleAdjuster(const std::vector<Shape2D>& shapes,
				const std::vector<std::vector<MatchInfo>>& pairwise_matches);

		bool estimate(std::vector<Camera>& cameras);

	protected:
		const std::vector<Shape2D>& shapes;
		const std::vector<std::vector<MatchInfo>>& pairwise_matches;
		const int nr_img;
		int nr_match;

		std::vector<double> params;

		double calcError(const std::vector<double>& params, std::vector<double>& err);
		void calcJacobian(Eigen::MatrixXd& J);
};

class IncrementalBundleAdjuster {
	public:
		IncrementalBundleAdjuster(
				const std::vector<Shape2D>& shapes,
				std::vector<Camera>& cameras):
			shapes(shapes), cameras(cameras), index_map(shapes.size())
		{ m_assert(shapes.size() == cameras.size()); }

		void add_match(int i, int j, const MatchInfo& m);

		void optimize();

	protected:
		const std::vector<Shape2D>& shapes;
		std::vector<Camera>& cameras;

		struct Term {
			int from, to;
			const MatchInfo& m;
			Term(int i, int j, const MatchInfo& m):
				from(i), to(j), m(m){}
		};

		int nr_pointwise_match = 0;
		std::vector<Term> terms;
		std::set<int> idx_added;
		// map from original image index to idx added
		std::vector<int> index_map;

		void update_index_map() {
			int cnt = 0;
			for (auto& i : idx_added)
				index_map[i] = cnt++;
		}

		struct ErrorStats {
			std::vector<double> term_err;
			double max, sum;
			ErrorStats(int size): term_err(size) {}

			void update_stats() {
				sum = max = 0;
				for (auto& e : term_err) {
					sum += sqr(e);
					update_max(max, fabs(e));
				}
				sum /= term_err.size();
				sum = sqrt(sum);
			}
		};

		struct ParamState {
			std::vector<Camera> cameras;
			std::vector<double> params;

			std::vector<Camera>& get_cameras();
			inline std::vector<double>& get_params() {
				ensure_params();
				return params;
			}
			void ensure_params();
		};

		ErrorStats calcError(ParamState& state);
		void calcJacobian(Eigen::MatrixXd& J, ParamState& state);


};

}
