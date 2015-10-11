//File: bundle_adjuster.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include "match_info.hh"
#include "lib/mat.h"
#include "camera.hh"
#include <vector>
#include <Eigen/Dense>


class BundleAdjuster {
	public:
		BundleAdjuster(const std::vector<Mat32f>& imgs,
				const std::vector<std::vector<MatchInfo>>& pairwise_matches);

		bool estimate(std::vector<Camera>& cameras);

	protected:
		const std::vector<Mat32f>& imgs;
		const std::vector<std::vector<MatchInfo>>& pairwise_matches;
		const int nr_img;
		int nr_match;

		std::vector<double> params;

		double calcError(const std::vector<double>& params, std::vector<double>& err);
		void calcJacobian(Eigen::MatrixXd& J);
};

class HomoEstimator {
	public:
		HomoEstimator(const MatchInfo& match_info);
		bool estimate(Homography& homo);

	protected:
		const MatchInfo& match;
		int nr_match;
		std::vector<double> params;
		double calcError(std::vector<double>& err);
		void calcJacobian(Eigen::MatrixXd& J);

};
