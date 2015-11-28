//File: bundle_adjuster.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "bundle_adjuster.hh"

#include <Eigen/Dense>
#include <cmath>
#include <memory>

#include "camera.hh"
#include "lib/timer.hh"
#include "match_info.hh"
using namespace std;
using namespace stitch;

namespace {
	const static int NR_PARAM_PER_CAMERA = 6;
	const static int NR_TERM_PER_MATCH = 2;
	const double LM_lambda = 0.05;
	const int LM_MAX_ITER = 100;

	void camera_to_params(const Camera& c, double* ptr) {
		ptr[0] = c.focal;
		ptr[1] = c.ppx;
		ptr[2] = c.ppy;
		Camera::rotation_to_angle(c.R, ptr[3], ptr[4], ptr[5]);
	}

	void params_to_camera(const double* ptr, Camera& c) {
		c.focal = ptr[0];
		c.ppx = ptr[1];
		c.ppy = ptr[2];
		c.aspect = 1;	// keep it 1
		Camera::angle_to_rotation(ptr[3], ptr[4], ptr[5], c.R);
	}
}

namespace stitch {
	// TODO incremental bundle adjustment, add in the order of confidence
	// TODO reject some connections if it breaks the bundle

	BundleAdjuster::BundleAdjuster(const vector<Shape2D>& shapes,
			const vector<vector<MatchInfo>>& pairwise_matches):
		shapes(shapes), pairwise_matches(pairwise_matches),
		nr_img(pairwise_matches.size()),
		nr_match(0),
		params(pairwise_matches.size() * NR_PARAM_PER_CAMERA)
	{
		REP(i, nr_img)
			REPL(j, i+1, nr_img) {
				auto& m = pairwise_matches[j][i];
				nr_match += m.match.size();
			}
	}

	bool BundleAdjuster::estimate(std::vector<Camera>& cameras) {
		using namespace Eigen;
		GuardedTimer tm("BundleAdjuster::estimate()");
		m_assert((int)cameras.size() == nr_img);
		REP(i, nr_img)
			camera_to_params(cameras[i], params.data() + i * NR_PARAM_PER_CAMERA);

		vector<double> err(NR_TERM_PER_MATCH * nr_match);
		double best_err = calcError(params, err);
		print_debug("BA: init err: %lf\n", best_err);
		abort();

		int itr = 0;
		int nr_non_decrease = 0;	// number of non-decreasing iteration
		while (itr++ < LM_MAX_ITER) {
			Eigen::MatrixXd J(NR_TERM_PER_MATCH * nr_match, NR_PARAM_PER_CAMERA * nr_img);
			calcJacobian(J);
			Eigen::MatrixXd JtJ = J.transpose() * J;
			REP(i, nr_img * NR_PARAM_PER_CAMERA)
				JtJ(i,i) += LM_lambda;	// TODO use different lambda for different param?
			Eigen::Map<VectorXd> err_vec(err.data(), NR_TERM_PER_MATCH * nr_match);
			auto b = J.transpose() * err_vec;
			VectorXd ans = JtJ.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

			vector<double> newparams = params;
			REP(i, newparams.size())
				newparams[i] += ans(i);
			double now_err = calcError(newparams, err);

			double max = -1e9; for (auto& e : err) update_max(max, abs(e));
			print_debug("BA: average err: %lf, max: %lf\n", now_err, max);
			if (now_err >= best_err - EPS)
				nr_non_decrease ++;
			else {
				nr_non_decrease = 0;
				best_err = now_err;
				params = move(newparams);
			}
			if (nr_non_decrease > 5)
				break;
		}
		print_debug("BA: Error %lf after %d iterations\n", best_err, itr);
		REP(i, nr_img)
			params_to_camera(params.data() + i * NR_PARAM_PER_CAMERA, cameras[i]);
		return true;
	}

	double BundleAdjuster::calcError(
			const vector<double>& params, std::vector<double>& err) {
		m_assert((int)err.size() == NR_TERM_PER_MATCH * nr_match);
		vector<Camera> now_camera(nr_img);
		REP(i, nr_img)
			params_to_camera(params.data() + i * NR_PARAM_PER_CAMERA, now_camera[i]);
		int idx = 0;
		REP(i, nr_img) REPL(j, i+1, nr_img) {
			auto& m = pairwise_matches[j][i];
			auto& c_from = now_camera[i], &c_to = now_camera[j];
			Homography Hto_to_from = (c_from.K() * c_from.R.transpose()) * (c_to.R * c_to.K().inverse());
			for (auto& p : m.match)	{
				Vec2D to = p.first, from = p.second;
				Vec2D transformed = Hto_to_from.trans2d(to + Vec2D(shapes[i].halfw(), shapes[i].halfh()));
				err[idx] = from.x - transformed.x + shapes[i].halfw();
				err[idx+1] = from.y - transformed.y + shapes[i].halfh();
				idx += 2;
			}
		}
		double sum = 0;
		for (auto& e : err) sum += sqr(e);
		sum /= err.size();
		sum = sqrt(sum);
		return sum;
	}

	void BundleAdjuster::calcJacobian(Eigen::MatrixXd& J) {
		const double step = 1e-5;
		vector<double> err1(NR_TERM_PER_MATCH * nr_match);
		vector<double> err2(NR_TERM_PER_MATCH * nr_match);
		REP(i, nr_img) {
			REP(p, NR_PARAM_PER_CAMERA) {
				int param_idx = i * NR_PARAM_PER_CAMERA + p;
				double val = params[param_idx];
				params[param_idx] = val + step;
				calcError(params, err1);
				params[param_idx] = val - step;
				calcError(params, err2);
				params[param_idx] = val;

				// deriv
				REP(k, err1.size())
					J(k, param_idx) = (err2[k] - err1[k]) / (2 * step);
			}
		}
	}

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
		PA(index_map);
		ParamState state;
		for (auto& idx : idx_added)
			state.cameras.emplace_back(cameras[idx]);
		state.ensure_params();
		state.cameras.clear();
		auto err_stat = calcError(state);
		double best_err = err_stat.sum;
		print_debug("BA: init err: %lf\n", err_stat.sum);

		int itr = 0;
		int nr_non_decrease = 0;// number of non-decreasing iteration
		int nr_img = idx_added.size();
		while (itr++ < LM_MAX_ITER) {
			Eigen::MatrixXd J(
					NR_TERM_PER_MATCH * nr_pointwise_match, NR_PARAM_PER_CAMERA * nr_img);
			calcJacobian(J, state);
			Eigen::MatrixXd JtJ = J.transpose() * J;
			REP(i, nr_img * NR_PARAM_PER_CAMERA)
				JtJ(i, i) += LM_lambda;	// TODO use different lambda for different param?
			Eigen::Map<VectorXd> err_vec(err_stat.term_err.data(), NR_TERM_PER_MATCH * nr_pointwise_match);
			auto b = J.transpose() * err_vec;
			VectorXd ans = JtJ.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

			ParamState new_state;
			new_state.params = state.get_params();
			REP(i, new_state.params.size())
				new_state.params[i] += ans(i);
			err_stat = calcError(new_state);
			print_debug("BA: average err: %lf, max: %lf\n", err_stat.sum, err_stat.max);

			if (err_stat.sum >= best_err - EPS)
				nr_non_decrease ++;
			else {
				nr_non_decrease = 0;
				best_err = err_stat.sum;
				state = move(new_state);
			}
			if (nr_non_decrease > 5)
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
				ret.term_err[idx] = from.x - transformed.x;
				ret.term_err[idx+1] = from.y - transformed.y;
				idx += 2;
			}
		}
		ret.update_stats();
		return ret;
	}

	void IncrementalBundleAdjuster::calcJacobian(
			Eigen::MatrixXd& J, ParamState& state) {
		const double step = 1e-5;
		state.ensure_params();
		REP(i, idx_added.size()) {
			REP(p, NR_PARAM_PER_CAMERA) {
				int param_idx = i * NR_PARAM_PER_CAMERA + p;
				double val = state.params[param_idx];
				state.params[param_idx] = val + step;
				state.cameras.clear();
				auto err1 = calcError(state);
				state.params[param_idx] = val - step;
				state.cameras.clear();
				auto err2 = calcError(state);
				state.params[param_idx] = val;
				state.cameras.clear();

				// calc deriv
				REP(k, err1.term_err.size())
					J(k, param_idx) = (err2.term_err[k] - err1.term_err[k]) / (2 * step);
			}
		}
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

}
