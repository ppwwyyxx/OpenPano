//File: ba_common.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <array>
#include "camera.hh"
#include "lib/timer.hh"

namespace {
	const static int NR_PARAM_PER_CAMERA = 6;
	const static int NR_TERM_PER_MATCH = 2;
	const double LM_lambda = 0.05;
	const int LM_MAX_ITER = 100;

	void camera_to_params(const stitch::Camera& c, double* ptr) {
		ptr[0] = c.focal;
		ptr[1] = c.ppx;
		ptr[2] = c.ppy;
		stitch::Camera::rotation_to_angle(c.R, ptr[3], ptr[4], ptr[5]);
	}

	void params_to_camera(const double* ptr, stitch::Camera& c) {
		c.focal = ptr[0];
		c.ppx = ptr[1];
		c.ppy = ptr[2];
		c.aspect = 1;	// keep it 1
		stitch::Camera::angle_to_rotation(ptr[3], ptr[4], ptr[5], c.R);
	}

	inline Homography cross_product_matrix(double x, double y, double z) {
		return Homography{(const double[]){0.0, -z, y, z, 0, -x, -y, x, 0}};
	}

	// See: http://arxiv.org/pdf/1312.0788.pdf
	// A compact formula for the derivative of a 3-D rotation in exponential coordinates
	// return 3 matrix, each is dR / dvi,
	// where vi is the component of the euler-vector of this R
	std::array<Homography, 3> dRdvi(const Homography& R) {
		TotalTimer tm("dRdvi");
		double v[3];
		stitch::Camera::rotation_to_angle(R, v[0], v[1], v[2]);
		Vec vvec{v[0], v[1], v[2]};
		double vsqr = vvec.sqr();
		if (vsqr < 1e-5)
			return std::array<Homography, 3>{cross_product_matrix(1, 0, 0), cross_product_matrix(0,1,0), cross_product_matrix(0,0,1)};
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

	// dR/dt1 = R * dRdt1
	static const Homography dRdt1((double[]){
			0.0, 0.0, 0.0,
			0.0, 0.0, -1.0,
			0.0, 1.0, 0.0});
	static const Homography dRdt2((double[]){
			0.0, 0.0, 1.0,
			0.0, 0.0, 0.0,
			-1.0, 0.0, 0.0});
	static const Homography dRdt3((double[]){
			0.0, -1.0, 0.0,
			1.0, 0.0, 0.0,
			0.0, 0.0, 0.0});
}
