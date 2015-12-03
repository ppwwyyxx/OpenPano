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

namespace {
inline Homography cross_product_matrix(double x, double y, double z) {
	return {(const double[]){0.0, -z, y, z, 0, -x, -y, x, 0}};
}

// See: http://arxiv.org/pdf/1312.0788.pdf
// A compact formula for the derivative of a 3-D rotation in exponential coordinates
// return 3 matrix, each is dR / dvi,
// where vi is the component of the euler-vector of this R
std::array<Homography, 3> dRdvi(const Homography& R) {
	double v[3];
	stitch::Camera::rotation_to_angle(R, v[0], v[1], v[2]);
	Vec vvec{v[0], v[1], v[2]};
	double vsqr = vvec.sqr();
	if (vsqr < GEO_EPS_SQR)
		return std::array<Homography, 3>{
				cross_product_matrix(1,0,0),
				cross_product_matrix(0,1,0),
				cross_product_matrix(0,0,1)};
	Homography r = cross_product_matrix(v[0], v[1], v[2]);
	std::array<Homography, 3> ret{r, r, r};
	REP(i, 3) ret[i].mult(v[i]);

	Vec I_R_e{1-R.data[0], -R.data[3], -R.data[6]};
	I_R_e = vvec.cross(I_R_e);
	ret[0] += cross_product_matrix(I_R_e.x, I_R_e.y, I_R_e.z);
	I_R_e = Vec{-R.data[1], 1-R.data[4], -R.data[7]};
	I_R_e = vvec.cross(I_R_e);
	ret[1] += cross_product_matrix(I_R_e.x, I_R_e.y, I_R_e.z);
	I_R_e = Vec{-R.data[2], -R.data[5], 1-R.data[8]};
	I_R_e = vvec.cross(I_R_e);
	ret[2] += cross_product_matrix(I_R_e.x, I_R_e.y, I_R_e.z);

	REP(i, 3) {
		ret[i].mult(1.0 / vsqr);
		ret[i] = ret[i] * R;
	}
	return ret;
}

// dK/dfocal = dKdfocal
static const Homography dKdfocal((const double[]){
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 0.0});
static const Homography dKdppx((const double[]){
		0.0, 0.0, 1.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0});
static const Homography dKdppy((const double[]){
		0.0, 0.0, 0.0,
		0.0, 0.0, 1.0,
		0.0, 0.0, 0.0});

}	// namespace

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
			Eigen::MatrixXd J(
					NR_TERM_PER_MATCH * nr_pointwise_match, NR_PARAM_PER_CAMERA * nr_img);
			calcJacobian(J, state);

			Eigen::MatrixXd JtJ(
					NR_PARAM_PER_CAMERA * nr_img, NR_PARAM_PER_CAMERA * nr_img);
			calcJtJ(JtJ, state);
			//auto JtJ = (J.transpose() * J).eval();

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
		if (not SYMBOLIC_DIFF) {
			// Numerical Differentiation of Residual w.r.t all parameters
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
		} else {
			// Symbolic Differentiation of Residual w.r.t all parameters
			// See Section 4 of: Automatic Panoramic Image Stitching using Invariant Features - David Lowe,IJCV07.pdf
			J.setZero();
			auto& cameras = state.get_cameras();
			// pre-calculate all derivatives of R
			vector<array<Homography, 3>> all_dRdvi;
			for (auto& c : cameras)
				all_dRdvi.emplace_back(dRdvi(c.R));

			int idx = 0;
			for (const auto& term: terms) {
				int from = index_map[term.from],
						to = index_map[term.to];
				int param_idx_from = from * NR_PARAM_PER_CAMERA,
						param_idx_to = to * NR_PARAM_PER_CAMERA;
				auto &c_from = cameras[from],
						 &c_to = cameras[to];
				auto toKinv = c_to.Kinv();
				auto toRinv = c_to.Rinv();
				const auto& dRfromdvi = all_dRdvi[from];
				auto dRtodviT = all_dRdvi[to];	// no copy here. will modify!
				for (auto& m: dRtodviT) m = m.transpose();

				Homography Hto_to_from = (c_from.K() * c_from.R) * (toRinv * toKinv);
				Vec2D mid_vec_to{shapes[term.to].halfw(), shapes[term.to].halfh()};

				for (const auto& p : term.m.match) {
					Vec2D to = p.first + mid_vec_to;
					Vec homo = Hto_to_from.trans(to);
					double hz_sqr = sqr(homo.z);
					double hz_inv = 1.0 / homo.z;

					auto set_J = [&](int param_idx, Vec dhdv /* d(homo) / d(variable) */) {
						// d(point 2d coor) / d(variable) = d(p)/d(homo) * d(homo)/d(variable)
						Vec2D dpdv{dhdv.x * hz_inv - dhdv.z * homo.x / hz_sqr,
							dhdv.y * hz_inv - dhdv.z * homo.y / hz_sqr};
						J(idx, param_idx) = -dpdv.x;	// d(residual) / d(variable) = -d(point) / d(variable)
						J(idx + 1, param_idx) = -dpdv.y;
					};

					// from:
					Homography m = c_from.R * toRinv * toKinv;
					Vec dot_u2 = m.trans(to);
					// focal
					set_J(param_idx_from, dKdfocal.trans(dot_u2));
					// ppx
					set_J(param_idx_from+1, dKdppx.trans(dot_u2));
					// ppy
					set_J(param_idx_from+2, dKdppy.trans(dot_u2));
					// rot
					m = c_from.K();
					dot_u2 = (toRinv * toKinv).trans(to);
					set_J(param_idx_from+3, (m * dRfromdvi[0]).trans(dot_u2));
					set_J(param_idx_from+4, (m * dRfromdvi[1]).trans(dot_u2));
					set_J(param_idx_from+5, (m * dRfromdvi[2]).trans(dot_u2));

					// to: d(Kinv) / dv = -Kinv * d(K)/dv * Kinv
					m = c_from.K() * c_from.R * toRinv * toKinv;
					dot_u2 = toKinv.trans(to) * (-1);
					// focal
					set_J(param_idx_to, (m * dKdfocal).trans(dot_u2));
					// ppx
					set_J(param_idx_to+1, (m * dKdppx).trans(dot_u2));
					// ppy
					set_J(param_idx_to+2, (m * dKdppy).trans(dot_u2));
					// rot
					m = c_from.K() * c_from.R;
					dot_u2 = toKinv.trans(to);
					set_J(param_idx_to+3, (m * dRtodviT[0]).trans(dot_u2));
					set_J(param_idx_to+4, (m * dRtodviT[1]).trans(dot_u2));
					set_J(param_idx_to+5, (m * dRtodviT[2]).trans(dot_u2));

					idx += 2;
				}
			}
		}
	}

	// XXX duplicate code should not happen!
	void IncrementalBundleAdjuster::calcJtJ(
			Eigen::MatrixXd& JtJ, ParamState& state) {
		TotalTimer tm("calcJtJ");
		JtJ.setZero();	// n_params * n_params
		auto& cameras = state.get_cameras();
		// pre-calculate all derivatives of R
		vector<array<Homography, 3>> all_dRdvi;
		for (auto& c : cameras)
			all_dRdvi.emplace_back(dRdvi(c.R));
		int idx = 0;
		for (const auto& term: terms) {
			int from = index_map[term.from],
					to = index_map[term.to];
			int param_idx_from = from * NR_PARAM_PER_CAMERA,
					param_idx_to = to * NR_PARAM_PER_CAMERA;
			auto &c_from = cameras[from],
					 &c_to = cameras[to];
			auto toKinv = c_to.Kinv();
			auto toRinv = c_to.Rinv();
			const auto& dRfromdvi = all_dRdvi[from];
			auto dRtodviT = all_dRdvi[to];	// no copy here. will modify!
			for (auto& m: dRtodviT) m = m.transpose();

			Homography Hto_to_from = (c_from.K() * c_from.R) * (toRinv * toKinv);
			Vec2D mid_vec_to{shapes[term.to].halfw(), shapes[term.to].halfh()};

			for (const auto& p : term.m.match) {
				Vec2D to = p.first + mid_vec_to;
				Vec homo = Hto_to_from.trans(to);
				double hz_sqr = sqr(homo.z);
				double hz_inv = 1.0 / homo.z;

				auto drdv = [&](Vec dhdv /* d(homo) / d(variable) */) {
					// d(point 2d coor) / d(variable) = d(p)/d(homo) * d(homo)/d(variable)
					Vec2D dpdv{dhdv.x * hz_inv - dhdv.z * homo.x / hz_sqr,
						dhdv.y * hz_inv - dhdv.z * homo.y / hz_sqr};
					return dpdv * (-1);
				};
				array<Vec2D, NR_PARAM_PER_CAMERA> dfrom, dto;

				// from:
				Homography m = c_from.R * toRinv * toKinv;
				Vec dot_u2 = m.trans(to);
				// focal
				dfrom[0] = drdv(dKdfocal.trans(dot_u2));
				// ppx
				dfrom[1] = drdv(dKdppx.trans(dot_u2));
				// ppy
				dfrom[2] = drdv(dKdppy.trans(dot_u2));
				// rot
				m = c_from.K();
				dot_u2 = (toRinv * toKinv).trans(to);
				dfrom[3] = drdv((m * dRfromdvi[0]).trans(dot_u2));
				dfrom[4] = drdv((m * dRfromdvi[1]).trans(dot_u2));
				dfrom[5] = drdv((m * dRfromdvi[2]).trans(dot_u2));

				// to: d(Kinv) / dv = -Kinv * d(K)/dv * Kinv
				m = c_from.K() * c_from.R * toRinv * toKinv;
				dot_u2 = toKinv.trans(to) * (-1);
				// focal
				dto[0] = drdv((m * dKdfocal).trans(dot_u2));
				// ppx
				dto[1] = drdv((m * dKdppx).trans(dot_u2));
				// ppy
				dto[2] = drdv((m * dKdppy).trans(dot_u2));
				// rot
				m = c_from.K() * c_from.R;
				dot_u2 = toKinv.trans(to);
				dto[3] = drdv((m * dRtodviT[0]).trans(dot_u2));
				dto[4] = drdv((m * dRtodviT[1]).trans(dot_u2));
				dto[5] = drdv((m * dRtodviT[2]).trans(dot_u2));
				// TODO duplicate!
				REP(i, 6) REP(j, 6) {
					int i1 = param_idx_from + i,
							i2 = param_idx_from + j;
					JtJ(i1, i2) += dfrom[i].dot(dfrom[j]);
					JtJ(i2, i1) += dfrom[i].dot(dfrom[j]);
					i1 = param_idx_to + i, i2 = param_idx_to + j;
					JtJ(i1, i2) += dto[i].dot(dto[j]);
					JtJ(i2, i1) += dto[i].dot(dto[j]);
					i1 = param_idx_from + i;
					i2 = param_idx_to + j;
					JtJ(i1, i2) += dfrom[i].dot(dto[j]);
					JtJ(i2, i1) += dfrom[i].dot(dto[j]);
					i1 = param_idx_to + i;
					i2 = param_idx_from + j;
					JtJ(i1, i2) += dto[i].dot(dfrom[j]);
					JtJ(i2, i1) += dto[i].dot(dfrom[j]);
				}

				idx += 2;
			}
		}

		JtJ *= 0.5;
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
