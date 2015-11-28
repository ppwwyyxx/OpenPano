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

#ifdef __has_cpp_attribute
#if __has_cpp_attribute(deprecated) >= 201309
// deprecated attribute used in lower version of gcc gives warnings.
class [[deprecated("use IncrementalBundleAdjuster instead")]] BundleAdjuster {
#else
class BundleAdjuster {
#endif
#else
class BundleAdjuster {
#endif
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

}
