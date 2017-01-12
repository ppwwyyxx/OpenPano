//File: incremental_bundle_adjuster.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#pragma once
#include <vector>
#include <set>
#include <Eigen/Dense>

#include "lib/mat.h"
#include "lib/utils.hh"
#include "common/common.hh"

namespace pano {
class Camera;
struct MatchInfo;
struct Shape2D;

class IncrementalBundleAdjuster {
	public:
		// statistics of error
		struct ErrorStats {
			std::vector<double> residuals;
			double max, avg;
			ErrorStats(int size): residuals(size) {}

			inline int num_terms() const { return residuals.size(); }

			void update_stats(int inlier_threshold);
		};

		IncrementalBundleAdjuster(
				std::vector<Camera>& cameras);

		IncrementalBundleAdjuster(const IncrementalBundleAdjuster&) = delete;
		IncrementalBundleAdjuster& operator = (const IncrementalBundleAdjuster&) = delete;

    // m is matches[j][i] in stitcher (i.e. from i to j)
		void add_match(int i, int j, const MatchInfo& m);

		void optimize();

		ErrorStats get_error_stat() {
			ParamState state;
			for (auto& c: result_cameras) state.cameras.emplace_back(c);
			state.ensure_params();
			return calcError(state);
		}

	protected:
		std::vector<Camera>& result_cameras;

		struct MatchPair {
			int from, to;		// the original image index
			const MatchInfo& m;
			MatchPair(int i, int j, const MatchInfo& m):
				from(i), to(j), m(m){}
		};

		int nr_pointwise_match = 0;
		int inlier_threshold = std::numeric_limits<int>::max();
		std::vector<MatchPair> match_pairs;

		// original indices that have appeared so far
		std::set<int> idx_added;

		// map from original image index to index being added
		std::vector<int> index_map;
		// map from index in match_pairs to the index of its first error term
		std::vector<int> match_cnt_prefix_sum;

		inline void update_index_map() {
			int cnt = 0;
			for (auto& i : idx_added)
				index_map[i] = cnt++;
		}

		// state of all parameters estimated so far
		// could be in the form of either cameras or params
		struct ParamState {
			std::vector<Camera> cameras;
			std::vector<double> params;

			std::vector<Camera>& get_cameras();
			const std::vector<Camera>& get_cameras() const {
				return const_cast<ParamState*>(this)->get_cameras();
			}

			void ensure_params() const;

			const std::vector<double>& get_params() const
			{ ensure_params(); return params; }

			std::vector<double>& get_params()
			{ ensure_params(); return params; }

			// change a param, and its corresponding camera
			void mutate_param(int param_idx, double new_val);
		};

		/// Optimization routines:
		Eigen::MatrixXd J, JtJ;		// to avoid too many malloc

		ErrorStats calcError(const ParamState& state);

		Eigen::VectorXd get_param_update(
				const ParamState& state, const std::vector<double>& residual, float);

		// calculate J & JtJ
		void calcJacobianNumerical(const ParamState& state);
		void calcJacobianSymbolic(const ParamState& state);

};

}
