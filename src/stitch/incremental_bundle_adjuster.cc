//File: incremental_bundle_adjuster.cc
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "incremental_bundle_adjuster.hh"

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <array>

#include "camera.hh"
#include "match_info.hh"
#include "lib/config.hh"
#include "projection.hh"
#include "lib/timer.hh"
using namespace std;
using namespace pano;
using namespace config;

namespace {
const static int NR_PARAM_PER_CAMERA = 6;
const static int NR_TERM_PER_MATCH = 2;
const static bool SYMBOLIC_DIFF = true;
const static int LM_MAX_ITER = 100;
const static float ERROR_IGNORE = 800.f;

inline void camera_to_params(const Camera& c, double* ptr) {
  ptr[0] = c.focal;
  ptr[1] = c.ppx;
  ptr[2] = c.ppy;
  Camera::rotation_to_angle(c.R, ptr[3], ptr[4], ptr[5]);
}

inline void params_to_camera(const double* ptr, Camera& c) {
  c.focal = ptr[0];
  c.ppx = ptr[1];
  c.ppy = ptr[2];
  c.aspect = 1;	// keep it 1
  Camera::angle_to_rotation(ptr[3], ptr[4], ptr[5], c.R);
}

inline Homography cross_product_matrix(double x, double y, double z) {
  return {{ 0 , -z, y,
    z , 0 , -x,
    -y, x , 0}};
}

// See: http://arxiv.org/pdf/1312.0788.pdf
// A compact formula for the derivative of a 3-D rotation in exponential coordinates
// return 3 matrix, each is dR / dvi,
// where vi is the component of the euler-vector of this R
std::array<Homography, 3> dRdvi(const Homography& R) {
  double v[3];
  Camera::rotation_to_angle(R, v[0], v[1], v[2]);
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
static const Homography dKdfocal({
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 0.0});
static const Homography dKdppx({
    0.0, 0.0, 1.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0});
static const Homography dKdppy({
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 0.0, 0.0});

}	// namespace

namespace pano {

IncrementalBundleAdjuster::IncrementalBundleAdjuster(
    std::vector<Camera>& cameras):
  result_cameras(cameras),
  index_map(cameras.size())
{ }


void IncrementalBundleAdjuster::add_match(
    int i, int j, const MatchInfo& match) {
  match_pairs.emplace_back(i, j, match);
  match_cnt_prefix_sum.emplace_back(nr_pointwise_match);
  nr_pointwise_match += match.match.size();
  idx_added.insert(i);
  idx_added.insert(j);
}

void IncrementalBundleAdjuster::optimize() {
  if (idx_added.empty())
    error_exit("Calling optimize() without adding any matches!");
  using namespace Eigen;
  update_index_map();
  int nr_img = idx_added.size();
  J = MatrixXd{NR_TERM_PER_MATCH * nr_pointwise_match, NR_PARAM_PER_CAMERA * nr_img};
  JtJ = MatrixXd{NR_PARAM_PER_CAMERA * nr_img, NR_PARAM_PER_CAMERA * nr_img};

  ParamState state;
  for (auto& idx : idx_added)
    state.cameras.emplace_back(result_cameras[idx]);
  state.ensure_params();
  state.cameras.clear();		// TODO why do I need this
  auto err_stat = calcError(state);
  double best_err = err_stat.avg;
  print_debug("BA: init err: %lf\n", best_err);

  int itr = 0;
  int nr_non_decrease = 0;// number of non-decreasing iteration
  inlier_threshold = std::numeric_limits<int>::max();
  while (itr++ < LM_MAX_ITER) {
    auto update = get_param_update(state, err_stat.residuals, LM_LAMBDA);

    ParamState new_state;
    new_state.params = state.get_params();
    REP(i, new_state.params.size())
      new_state.params[i] -= update(i);
    err_stat = calcError(new_state);
    print_debug("BA: average err: %lf, max: %lf\n", err_stat.avg, err_stat.max);

    if (err_stat.avg >= best_err - 1e-3)
      nr_non_decrease ++;
    else {
      nr_non_decrease = 0;
      best_err = err_stat.avg;
      state = move(new_state);
    }
    if (nr_non_decrease > 5)
      break;
  }
  print_debug("BA: Error %lf after %d iterations\n", best_err, itr);

  auto results = state.get_cameras();
  int now = 0;
  for (auto& i : idx_added)
    result_cameras[i] = results[now++];
}

IncrementalBundleAdjuster::ErrorStats IncrementalBundleAdjuster::calcError(
    const ParamState& state) {
  ErrorStats ret(nr_pointwise_match * NR_TERM_PER_MATCH);
  auto cameras = state.get_cameras();

  int idx = 0;
  for (auto& pair: match_pairs) {
    int from = index_map[pair.from],
    to = index_map[pair.to];
    auto& c_from = cameras[from],
    & c_to = cameras[to];
    Homography Hto_to_from = (c_from.K() * c_from.R) *
      (c_to.Rinv() * c_to.K().inverse());

    for (const auto& p: pair.m.match) {
      Vec2D to = p.first, from = p.second;
      // we are estimating Hs that work on [-w/2,w/2] coordinate
      Vec2D transformed = Hto_to_from.trans2d(to);
      ret.residuals[idx] = from.x - transformed.x;
      ret.residuals[idx+1] = from.y - transformed.y;

      // TODO for the moment, ignore circlic error
      /*
       *if (fabs(ret.residuals[idx]) > ERROR_IGNORE) {
       *  auto transfed = spherical::homo2proj(Hto_to_from.trans(to));
       *  auto from3d = spherical::homo2proj(Vec{from.x, from.y, 1});
       *  PP(from);PP(transformed);
       *  PP(transfed); PP(from3d);
       *  //ret.residuals[idx] = 0;
       *}
       */
      idx += 2;
    }
  }
  ret.update_stats(inlier_threshold);
  return ret;
}

void IncrementalBundleAdjuster::ErrorStats::update_stats(int) {
  // TODO which error func to use?
  auto error_func = [&](double diff) -> double {
    return sqr(diff);	// square error is good
    /*
     *diff = fabs(diff);
     *if (diff < inlier_threshold)
     *  return sqr(diff);
     *return 2.0 * inlier_threshold * diff - sqr(inlier_threshold);
     */
  };

  avg = max = 0;
  for (auto& e : residuals) {
    avg += error_func(e);
    update_max(max, fabs(e));
    //if (update_max(max, fabs(e))) PP(e);
  }
  avg /= residuals.size();
  avg = sqrt(avg);

}

Eigen::VectorXd IncrementalBundleAdjuster::get_param_update(
    const ParamState& state, const vector<double>& residual, float lambda) {
  TotalTimer tm("get_param_update");
  using namespace Eigen;
  int nr_img = idx_added.size();
  if (! SYMBOLIC_DIFF) {
    calcJacobianNumerical(state);
  } else {
    //calcJacobianNumerical(state);
    //auto Jcopy = J;
    calcJacobianSymbolic(state);
    // check correctness
    //PP((JtJ - J.transpose() * J).eval().maxCoeff());
    //PP((J - Jcopy).eval().maxCoeff());
  }
  Map<const VectorXd> err_vec(residual.data(), NR_TERM_PER_MATCH * nr_pointwise_match);
  auto b = J.transpose() * err_vec;

  REP(i, nr_img * NR_PARAM_PER_CAMERA) {
    // use different lambda for different param? from Lowe.
    // *= (1+lambda) ?
    if (i % NR_PARAM_PER_CAMERA >= 3) {
      JtJ(i, i) += lambda;
    } else {
      JtJ(i, i) += lambda / 10.f;
    }
  }
  //return JtJ.partialPivLu().solve(b).eval();
  return JtJ.colPivHouseholderQr().solve(b).eval();
}

void IncrementalBundleAdjuster::calcJacobianNumerical(const ParamState& old_state) {
  TotalTimer tm("calcJacobianNumerical");
  // Numerical Differentiation of Residual w.r.t all parameters
  const static double step = 1e-6;
  ParamState& state = const_cast<ParamState&>(old_state); // all mutated state will be recovered at last.
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
      REP(k, err1.num_terms())
        J(k, param_idx) = (err1.residuals[k] - err2.residuals[k]) / (2 * step);
    }
  }
  JtJ = (J.transpose() * J).eval();
}

void IncrementalBundleAdjuster::calcJacobianSymbolic(const ParamState& state) {
  // Symbolic Differentiation of Residual w.r.t all parameters
  // See Section 4 of: Automatic Panoramic Image Stitching using Invariant Features - David Lowe,IJCV07.pdf
  TotalTimer tm("calcJacobianSymbolic");
  J.setZero();	// this took 1/3 time. J.rows() could reach 700000 sometimes.
  JtJ.setZero();
  const auto& cameras = state.get_cameras();
  // pre-calculate all derivatives of R
  vector<array<Homography, 3>> all_dRdvi(cameras.size());
  REP(i, cameras.size())
    all_dRdvi[i] = dRdvi(cameras[i].R);

  REP(pair_idx, match_pairs.size()) {
    const MatchPair& pair = match_pairs[pair_idx];
    int idx = match_cnt_prefix_sum[pair_idx] * 2;
    int from = index_map[pair.from],
    to = index_map[pair.to];
    int param_idx_from = from * NR_PARAM_PER_CAMERA,
        param_idx_to = to * NR_PARAM_PER_CAMERA;
    const auto &c_from = cameras[from],
    &c_to = cameras[to];
    const auto fromK = c_from.K();
    const auto toKinv = c_to.Kinv();
    const auto toRinv = c_to.Rinv();
    const auto& dRfromdvi = all_dRdvi[from];
    auto dRtodviT = all_dRdvi[to];	// copying. will modify!
    for (auto& m: dRtodviT) m = m.transpose();

    const Homography Hto_to_from = (fromK * c_from.R) * (toRinv * toKinv);

    for (const auto& p : pair.m.match) {
      Vec2D to = p.first;//, from = p.second;
      Vec homo = Hto_to_from.trans(to);
      double hz_sqr_inv = 1.0 / sqr(homo.z);
      double hz_inv = 1.0 / homo.z;

      // TODO use spherical projection instead of flat projection
      /*
       *if (fabs(from.x - homo.x / homo.z) > ERROR_IGNORE) {
       *  REP(i, 6) {
       *    J(idx, param_idx_from+i) = 0;
       *    J(idx, param_idx_to+i) = 0;
       *    J(idx+1, param_idx_from+i) = 0;
       *    J(idx+1, param_idx_to+i) = 0;
       *  }
       *  idx += 2;
       *  continue;
       *}
       */

      Vec dhdv;	// d(homo)/d(variable)
      // calculate d(residual) / d(variable) = -d(point 2d) / d(variable)
      // d(point 2d coor) / d(variable) = d(p2d)/d(homo3d) * d(homo3d)/d(variable)
#define drdv(xx) \
      (dhdv = xx, Vec2D{ \
       -dhdv.x * hz_inv + dhdv.z * homo.x * hz_sqr_inv, \
       -dhdv.y * hz_inv + dhdv.z * homo.y * hz_sqr_inv})
//#define drdv(xx) -flat::gradproj(homo, xx)

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
      dot_u2 = (toRinv * toKinv).trans(to);
      dfrom[3] = drdv((fromK * dRfromdvi[0]).trans(dot_u2));
      dfrom[4] = drdv((fromK * dRfromdvi[1]).trans(dot_u2));
      dfrom[5] = drdv((fromK * dRfromdvi[2]).trans(dot_u2));

      // to: d(Kinv) / dv = -Kinv * d(K)/dv * Kinv
      m = fromK * c_from.R * toRinv * toKinv;
      dot_u2 = toKinv.trans(to) * (-1);
      // focal
      dto[0] = drdv((m * dKdfocal).trans(dot_u2));
      // ppx
      dto[1] = drdv((m * dKdppx).trans(dot_u2));
      // ppy
      dto[2] = drdv((m * dKdppy).trans(dot_u2));
      // rot
      m = fromK * c_from.R;
      dot_u2 = toKinv.trans(to);
      dto[3] = drdv((m * dRtodviT[0]).trans(dot_u2));
      dto[4] = drdv((m * dRtodviT[1]).trans(dot_u2));
      dto[5] = drdv((m * dRtodviT[2]).trans(dot_u2));
#undef drdv

      // fill J
      REP(i, 6) {
        J(idx, param_idx_from+i) = dfrom[i].x;
        J(idx, param_idx_to+i) = dto[i].x;
        J(idx+1, param_idx_from+i) = dfrom[i].y;
        J(idx+1, param_idx_to+i) = dto[i].y;
      }

      // fill JtJ
      REP(i, 6) REP(j, 6) {
        int i1 = param_idx_from + i,
            i2 = param_idx_to + j;
        auto val = dfrom[i].dot(dto[j]);
        JtJ(i1, i2) += val, JtJ(i2, i1) += val;
      }
      REP(i, 6) REPL(j, i, 6) {
        int i1 = param_idx_from + i,
            i2 = param_idx_from + j;
        auto val = dfrom[i].dot(dfrom[j]);
        JtJ(i1, i2) += val;
        if (i != j) JtJ(i2, i1) += val;

        i1 = param_idx_to + i, i2 = param_idx_to + j;
        val = dto[i].dot(dto[j]);
        JtJ(i1, i2) += val;
        if (i != j) JtJ(i2, i1) += val;
      }
      idx += 2;
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

void IncrementalBundleAdjuster::ParamState::ensure_params() const {
  if (params.size())
    return;
  m_assert(cameras.size());
  // this function serves the semantic of 'updating the cache'. thus it is const
  vector<double>& params = const_cast<vector<double>&>(this->params);
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
