//File: incremental_bundle_adjuster.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "incremental_bundle_adjuster.hh"
#include "ba_common.hh"

#include <Eigen/Dense>
#include <cmath>
#include <memory>

#include "camera.hh"
#include "lib/timer.hh"
#include "match_info.hh"
using namespace std;
using namespace stitch;

namespace stitch {
	void IncrementalBundleAdjuster::add_match(
			int i, int j, const MatchInfo& match) {
		terms.emplace_back(i, j, match);
		nr_pointwise_match += match.match.size();
		idx_added.insert(i);
		idx_added.insert(j);
	}

	void IncrementalBundleAdjuster::optimize() {
		using namespace Eigen;
		update_index_map();
		ParamState state;
		for (auto& idx : idx_added)
			state.cameras.emplace_back(cameras[idx]);
		state.ensure_params();
		state.cameras.clear();
		auto err_stat = calcError(state);
		double best_err = err_stat.avg;
		print_debug("BA: init err: %lf\n", best_err);

		int itr = 0;
		int nr_non_decrease = 0;// number of non-decreasing iteration
		int nr_img = idx_added.size();
		inlier_threshold = std::numeric_limits<int>::max();
		while (itr++ < LM_MAX_ITER) {
			if (itr > 2 && inlier_threshold != 2) {
				inlier_threshold = 2;
				best_err = 1e9;
			}
			Eigen::MatrixXd J(
					NR_TERM_PER_MATCH * nr_pointwise_match, NR_PARAM_PER_CAMERA * nr_img);
			calcJacobian(J, state);
			Eigen::MatrixXd JtJ = J.transpose() * J;
			REP(i, nr_img * NR_PARAM_PER_CAMERA)
				JtJ(i, i) += LM_lambda;	// TODO use different lambda for different param?
			Eigen::Map<VectorXd> err_vec(err_stat.residuals.data(), NR_TERM_PER_MATCH * nr_pointwise_match);
			auto b = J.transpose() * err_vec;
			VectorXd ans = JtJ.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

			ParamState new_state;
			new_state.params = state.get_params();
			REP(i, new_state.params.size())
				new_state.params[i] -= ans(i);
			err_stat = calcError(new_state);
			print_debug("BA: average err: %lf, max: %lf\n", err_stat.avg, err_stat.max);

			if (err_stat.avg >= best_err - 1e-4)
				nr_non_decrease ++;
			else {
				nr_non_decrease = 0;
				best_err = err_stat.avg;
				state = move(new_state);
			}
			if (nr_non_decrease > 3)
				break;
		}
		print_debug("BA: Error %lf after %d iterations\n", best_err, itr);

		auto results = state.get_cameras();
		int now = 0;
		for (auto& i : idx_added)
			cameras[i] = results[now++];
	}

	IncrementalBundleAdjuster::ErrorStats IncrementalBundleAdjuster::calcError(
			ParamState& state) {
		TotalTimer tm("calcError");
		ErrorStats ret(nr_pointwise_match * NR_TERM_PER_MATCH);
		auto cameras = state.get_cameras();

		int idx = 0;
		for (auto& term: terms) {
			int from = index_map[term.from],
					to = index_map[term.to];
			auto& c_from = cameras[from],
				& c_to = cameras[to];
			Homography Hto_to_from = (c_from.K() * c_from.R.transpose()) * (c_to.R * c_to.K().inverse());

			Vec2D mid_vec_from{shapes[term.from].halfw(), shapes[term.from].halfh()};
			Vec2D mid_vec_to{shapes[term.to].halfw(), shapes[term.to].halfh()};
			for (auto& p: term.m.match) {
				Vec2D to = p.first + mid_vec_to, from = p.second + mid_vec_from;
				Vec2D transformed = Hto_to_from.trans2d(to);
				ret.residuals[idx] = from.x - transformed.x;
				ret.residuals[idx+1] = from.y - transformed.y;
				idx += 2;
			}
		}
		ret.update_stats(inlier_threshold);
		return ret;
	}

	void IncrementalBundleAdjuster::calcJacobian(
			Eigen::MatrixXd& J, ParamState& state) {
		TotalTimer tm("calcJacobian");
		const double step = 1e-5;
		state.ensure_params();
		REP(i, idx_added.size()) {
			REP(p, NR_PARAM_PER_CAMERA) {
				int param_idx = i * NR_PARAM_PER_CAMERA + p;
				double val = state.params[param_idx];
				state.mutate_param(param_idx, val + step);
				auto err1 = calcError(state);
				state.mutate_param(param_idx, val - step);
				auto err2 = calcError(state);
				state.mutate_param(param_idx, val);

				// calc deriv
				REP(k, err1.residuals.size())
					J(k, param_idx) = (err1.residuals[k] - err2.residuals[k]) / (2 * step);
			}
		}


/*
 *    auto test_jacobian = [&]() {
 *      auto& cameras = state.get_cameras();
 *
 *      int param_idx = 9;
 *      auto oldc = cameras[1].Rinv();
 *      double val = state.params[param_idx];
 *      state.mutate_param(param_idx, val + step);
 *      auto c1 = state.get_cameras()[1].Rinv();
 *      state.mutate_param(param_idx, val - step);
 *      auto c2 = state.get_cameras()[1].Rinv();
 *      PP(oldc * dRdt1);
 *      auto diff = c1 - c2;
 *      diff.mult(0.5 / step);
 *      PP(diff);
 *      return;
 *
 *      int idx = 0;
 *      for (auto& term: terms)	{
 *        int from = index_map[term.from],
 *            to = index_map[term.to];
 *        int param_idx_from = from * NR_PARAM_PER_CAMERA,
 *            param_idx_to = to * NR_PARAM_PER_CAMERA;
 *        auto& c_from = cameras[from],
 *          & c_to = cameras[to];
 *        auto toKinv = c_to.K().inverse();
 *        Homography Hto_to_from = (c_from.K() * c_from.R.transpose()) * (c_to.R * toKinv);
 *
 *        Vec2D mid_vec_to{shapes[term.to].halfw(), shapes[term.to].halfh()};
 *
 *        auto dpdhomo = [](Vec homo, Vec d) {
 *          double hz_sqr = sqr(homo.z);
 *          return Vec2D(d.x / homo.z - d.z * homo.x / hz_sqr,
 *              d.y / homo.z - d.z * homo.y / hz_sqr);
 *        };
 *
 *        for (auto& p : term.m.match) {
 *          PP(J.row(idx));
 *          PP(J.row(idx+1));
 *          Vec2D to = p.first + mid_vec_to;
 *          Vec homo = Hto_to_from.trans(to);
 *
 *          // from:
 *          Homography m = c_from.R.transpose() * c_to.R * toKinv;
 *          Vec dot_u2 = m.trans(to);
 *          // focal
 *          Vec dhdfocal = dKdfocal.trans(dot_u2);
 *          Vec2D drv = dpdhomo(homo, dhdfocal) * (-1);
 *          J(idx, param_idx_from) = drv.x;
 *          J(idx+1, param_idx_from) = drv.y;
 *          PP(drv);
 *          // ppx
 *          Vec dhdppx = dKdppx.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdppx) * (-1);
 *          J(idx, param_idx_from+1) = drv.x;
 *          J(idx+1, param_idx_from+1) = drv.y;
 *          PP(drv);
 *          // ppy
 *          Vec dhdppy = dKdppy.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdppy) * (-1);
 *          J(idx, param_idx_from+2) = drv.x;
 *          J(idx+1, param_idx_from+2) = drv.y;
 *          PP(drv);
 *
 *          m = c_from.K() * c_from.R.transpose();
 *          dot_u2 = Homography{c_to.R * toKinv}.trans(to);
 *          // t1
 *          Vec dhdt1 = Homography{m * dRdt1}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdt1);
 *          J(idx, param_idx_from+3) = drv.x;
 *          J(idx+1, param_idx_from+3) = drv.y;
 *          PP(drv);
 *          // t2
 *          Vec dhdt2 = Homography{m * dRdt2}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdt2);
 *          J(idx, param_idx_from+4) = drv.x;
 *          J(idx+1, param_idx_from+4) = drv.y;
 *          PP(drv);
 *          // t3
 *          Vec dhdt3 = Homography{m * dRdt3}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdt3);
 *          J(idx, param_idx_from+5) = drv.x;
 *          J(idx+1, param_idx_from+5) = drv.y;
 *          PP(drv);
 *
 *          // to:
 *          m = c_from.K() * c_from.R.transpose() * c_to.R * toKinv;
 *          dot_u2 = toKinv.trans(to);
 *          // focal
 *          dhdfocal = Homography{m * dKdfocal}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdfocal);
 *          J(idx, param_idx_to) = drv.x;
 *          J(idx+1, param_idx_to) = drv.y;
 *          PP(drv);
 *          // ppx
 *          dhdppx = Homography{m * dKdppx}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdppx);
 *          J(idx, param_idx_to+1) = drv.x;
 *          J(idx+1, param_idx_to+1) = drv.y;
 *          PP(drv);
 *          // ppy
 *          dhdppy = Homography{m * dKdppy}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdppy);
 *          J(idx, param_idx_to+2) = drv.x;
 *          J(idx+1, param_idx_to+2) = drv.y;
 *          PP(drv);
 *
 *          // XXX wrong?
 *          m = c_from.K() * (c_from.R.transpose());
 *          dot_u2 = Homography{c_to.R * toKinv}.trans(to);
 *          // t1
 *          dhdt1 = Homography{m * (dRdt1.transpose())}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdt1);
 *          J(idx, param_idx_to+3) = drv.x;
 *          J(idx+1, param_idx_to+3) = drv.y;
 *          PP(drv);
 *          // t2
 *          dhdt2 = Homography{m * (dRdt2.transpose())}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdt2);
 *          J(idx, param_idx_to+4) = drv.x;
 *          J(idx+1, param_idx_to+4) = drv.y;
 *          PP(drv);
 *          // t3
 *          dhdt3 = Homography{m * (dRdt3.transpose())}.trans(dot_u2);
 *          drv = dpdhomo(homo, dhdt3);
 *          J(idx, param_idx_to+5) = drv.x;
 *          J(idx+1, param_idx_to+5) = drv.y;
 *          PP(drv);
 *
 *          idx += 2;
 *
 *          cout << "---------------------" << endl;
 *        }
 *        //break;
 *      }
 *    };
 *    test_jacobian();
 */
	}

	vector<Camera>& IncrementalBundleAdjuster::ParamState::get_cameras() {
		if (cameras.size())
			return cameras;
		m_assert(params.size());
		cameras.resize(params.size() / NR_PARAM_PER_CAMERA);
		REP(i, cameras.size())
			params_to_camera(params.data() + i * NR_PARAM_PER_CAMERA, cameras[i]);
		return cameras;
	}

	void IncrementalBundleAdjuster::ParamState::ensure_params() {
		if (params.size())
			return;
		m_assert(cameras.size());
		params.resize(cameras.size() * NR_PARAM_PER_CAMERA);
		REP(i, cameras.size())
			camera_to_params(cameras[i], params.data() + i * NR_PARAM_PER_CAMERA);
	}

	void IncrementalBundleAdjuster::ParamState::mutate_param(int param_idx, double new_val) {
		ensure_params();
		auto& cameras = get_cameras();
		int camera_id = param_idx / NR_PARAM_PER_CAMERA;
		params[param_idx] = new_val;
		params_to_camera(
				params.data() + camera_id * NR_PARAM_PER_CAMERA,
				cameras[camera_id]);
	}
}
