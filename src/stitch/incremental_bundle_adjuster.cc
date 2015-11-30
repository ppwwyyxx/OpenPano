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
			state.cameras.emplace_back(result_cameras[idx]);
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
			/*
			 *if (itr > 2 && inlier_threshold != 2) {
			 *  inlier_threshold = 2;
			 *  best_err = 1e9;
			 *}
			 */
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
			result_cameras[i] = results[now++];
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
			Homography Hto_to_from = (c_from.K() * c_from.R) * (c_to.Rinv() * c_to.K().inverse());

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
#if 0
		const double step = 1e-6;
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
#else
		J.setZero();

		auto& cameras = state.get_cameras();
		vector<array<Homography, 3>> all_camera_dRdvi;
		for (auto& c : cameras)
			all_camera_dRdvi.emplace_back(dRdvi(c.R));

		int idx = 0;
		for (auto& term: terms)	{
			int from = index_map[term.from],
					to = index_map[term.to];
			int param_idx_from = from * NR_PARAM_PER_CAMERA,
					param_idx_to = to * NR_PARAM_PER_CAMERA;
			auto& c_from = cameras[from],
				& c_to = cameras[to];
			auto toKinv = c_to.Kinv();
			auto toRinv = c_to.Rinv();
			auto& dRfromdvi = all_camera_dRdvi[from];
			auto& dRtodvi = all_camera_dRdvi[to];
			Homography Hto_to_from = (c_from.K() * c_from.R) * (toRinv * toKinv);

			Vec2D mid_vec_to{shapes[term.to].halfw(), shapes[term.to].halfh()};

			auto dpdhomo = [](Vec homo, Vec d /* d(homo) / d(variable) */) {		// d(2d point) / d(homo coordiante point)
				double hz_sqr = sqr(homo.z);
				return Vec2D(d.x / homo.z - d.z * homo.x / hz_sqr,
						d.y / homo.z - d.z * homo.y / hz_sqr);
			};

			for (auto& p : term.m.match) {
				Vec2D to = p.first + mid_vec_to;
				Vec homo = Hto_to_from.trans(to);
				Vec2D drv; Vec dhdv;

				auto set_J = [&](int param_idx) {
					drv = dpdhomo(homo, dhdv);
					/*
					 *if (fabs(J(idx, param_idx) + drv.x) > 0.1 || fabs(J(idx+1, param_idx) + drv.y) > 0.1) {
					 *  int diff = param_idx - param_idx_to;
					 *  bool is_to = (diff >= 0 && diff <= 5);
					 *    PP(param_idx);
					 *    PP(drv);
					 *    PP(J(idx, param_idx));
					 *    PP(J(idx+1, param_idx));
					 *  if (is_to) {
					 *    PP(c_to.R);
					 *    PP(dRtodvi[1]);
					 *  } else {
					 *    PP(c_from.R);
					 *    PP(dRfromdvi[1]);
					 *  }
					 *}
					 */
					J(idx, param_idx) = -drv.x;
					J(idx+1, param_idx) = -drv.y;
				};

				// from:
				Homography m = c_from.R * toRinv * toKinv;
				Vec dot_u2 = m.trans(to);
				// focal
				dhdv = dKdfocal.trans(dot_u2);	// d(homo) / d(variable)
				set_J(param_idx_from);
				// ppx
				dhdv = dKdppx.trans(dot_u2);
				set_J(param_idx_from+1);
				// ppy
				dhdv = dKdppy.trans(dot_u2);
				set_J(param_idx_from+2);

				m = c_from.K();
				dot_u2 = Homography{toRinv * toKinv}.trans(to);
				// t1
				dhdv = Homography{m * dRfromdvi[0]}.trans(dot_u2);
				set_J(param_idx_from+3);
				// t2
				dhdv = Homography{m * dRfromdvi[1]}.trans(dot_u2);
				set_J(param_idx_from+4);
				// t3
				dhdv = Homography{m * dRfromdvi[2]}.trans(dot_u2);
				set_J(param_idx_from+5);

				// to: d(Kinv) / dv = -Kinv * d(K)/dv * Kinv
				m = c_from.K() * c_from.R * toRinv * toKinv;
				dot_u2 = toKinv.trans(to) * (-1);
				// focal
				dhdv = Homography{m * dKdfocal}.trans(dot_u2);
				set_J(param_idx_to);
				// ppx
				dhdv = Homography{m * dKdppx}.trans(dot_u2);
				set_J(param_idx_to+1);
				// ppy
				dhdv = Homography{m * dKdppy}.trans(dot_u2);
				set_J(param_idx_to+2);

				m = c_from.K() * c_from.R;
				dot_u2 = toKinv.trans(to);
				// t1
				dhdv = Homography{m * dRtodvi[0].transpose()}.trans(dot_u2);
				set_J(param_idx_to+3);
				// t2
				dhdv = Homography{m * dRtodvi[1].transpose()}.trans(dot_u2);
				set_J(param_idx_to+4);
				// t3
				dhdv = Homography{m * dRtodvi[2].transpose()}.trans(dot_u2);
				set_J(param_idx_to+5);

				idx += 2;
			}
		}
#endif
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
