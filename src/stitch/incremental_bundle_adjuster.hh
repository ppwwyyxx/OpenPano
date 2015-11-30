//File: incremental_bundle_adjuster.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

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

class IncrementalBundleAdjuster {
	public:
		IncrementalBundleAdjuster(
				const std::vector<Shape2D>& shapes,
				std::vector<Camera>& cameras):
			shapes(shapes), result_cameras(cameras), index_map(shapes.size())
		{ m_assert(shapes.size() == cameras.size()); }

		void add_match(int i, int j, const MatchInfo& m);

		void optimize();

	protected:
		const std::vector<Shape2D>& shapes;
		std::vector<Camera>& result_cameras;

		struct Term {
			int from, to;
			const MatchInfo& m;
			Term(int i, int j, const MatchInfo& m):
				from(i), to(j), m(m){}
		};

		int nr_pointwise_match = 0;
		int inlier_threshold = std::numeric_limits<int>::max();
		std::vector<Term> terms;

		std::set<int> idx_added;

		// map from original image index to idx added
		std::vector<int> index_map;

		inline void update_index_map() {
			int cnt = 0;
			for (auto& i : idx_added)
				index_map[i] = cnt++;
		}

		struct ErrorStats {
			std::vector<double> residuals;
			double max, avg, cost;
			ErrorStats(int size): residuals(size) {}

			int num_terms() const { return residuals.size(); }

			void update_stats(int inlier_threshold) {
				auto error_func = [&](double diff) -> double {
					return sqr(diff);	// square error is good
					diff = fabs(diff);
					if (diff < inlier_threshold)
						return sqr(diff);
					return 2.0 * inlier_threshold * diff - sqr(inlier_threshold);
				};

				avg = max = 0;
				for (auto& e : residuals) {
					avg += error_func(e);
					update_max(max, fabs(e));
				}
				avg /= residuals.size();
				avg = sqrt(avg);
			}
		};

		struct ParamState {
			std::vector<Camera> cameras;
			std::vector<double> params;

			std::vector<Camera>& get_cameras();
			void ensure_params();

			inline std::vector<double>& get_params() {
				ensure_params();
				return params;
			}

			void mutate_param(int param_idx, double new_val);
		};

		ErrorStats calcError(ParamState& state);
		void calcJacobian(Eigen::MatrixXd& J, ParamState& state);

};

}
