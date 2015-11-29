//File: ba_common.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "camera.hh"

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

	// A compact formula for the derivative of a 3-D rotation in exponential coordinates
	Matrix dRdvi(const Matrix& R, int i) {
		double v[3];
		stitch::Camera::rotation_to_angle(R, v[0], v[1], v[2]);
		Vec vvec{v[0], v[1], v[2]};
		double vsqr = vvec.sqr();
		if (vsqr < 1e-5)
			return cross_product_matrix(i==0, i==1, i==2);
		Matrix ret = cross_product_matrix(v[0], v[1], v[2]);
		ret.mult(v[i]);
		Vec I_R_e{(double)(i==0) - R.at(0,i),
							(double)(i==1) - R.at(1,i),
							(double)(i==2) - R.at(2,i)};
		I_R_e = vvec.cross(I_R_e);
		ret = ret + cross_product_matrix(I_R_e.x, I_R_e.y, I_R_e.z);
		ret.mult(1.0 / vsqr);
		ret = ret * R;
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
