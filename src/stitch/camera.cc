//File: camera.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "camera.hh"

#include <Eigen/Dense>
#include <algorithm>
#include <string>

#include "lib/timer.hh"
#include "match_info.hh"
#include "transform.hh"
using namespace std;
// Implement stuffs about camera K,R matrices
// Mostly copy from OpenCV for stability

namespace {

// From Creating Full View Panoramic Image Mosaics - Szeliski
double get_focal_from_matrix(const Matrix& m) {
	const double* h = m.ptr();

	double d1, d2; // Denominators
	double v1, v2; // Focal squares value candidates
	double f1, f0;

	d1 = h[6] * h[7];
	d2 = (h[7] - h[6]) * (h[7] + h[6]);
	v1 = -(h[0] * h[1] + h[3] * h[4]) / d1;
	v2 = (h[0] * h[0] + h[3] * h[3] - h[1] * h[1] - h[4] * h[4]) / d2;
	if (v1 < v2)
		swap(v1, v2);
	if (v1 > 0 && v2 > 0)
		f1 = sqrt(abs(d1) > abs(d2) ? v1 : v2);
	else if (v1 > 0)
		f1 = sqrt(v1);
	else
		return 0;

	d1 = h[0] * h[3] + h[1] * h[4];
	d2 = h[0] * h[0] + h[1] * h[1] - h[3] * h[3] - h[4] * h[4];
	v1 = -h[2] * h[5] / d1;
	v2 = (h[5] * h[5] - h[2] * h[2]) / d2;
	if (v1 < v2)
		swap(v1, v2);
	if (v1 > 0 && v2 > 0)
		f0 = sqrt(abs(d1) > abs(d2) ? v1 : v2);
	else if (v1 > 0)
		f0 = sqrt(v1);
	else
		return 0;
	if (std::isinf(f1) || std::isinf(f0))
		return 0;
	return sqrt(f1 * f0);
}

}

namespace stitch {

Camera::Camera() : R(Homography::I()) { }

Homography Camera::K() const {
	Homography ret(Homography::I());
	ret.at(0,0) = focal;
	ret.at(0,2) = ppx;
	ret.at(1,1) = focal * aspect;
	ret.at(1,2) = ppy;
	return ret;
}

double Camera::estimate_focal(
		const vector<vector<MatchInfo>>& matches) {
	int n = matches.size();
	vector<double> estimates;
	REP(i, n) REPL(j, i + 1, n) {
		auto& match = matches[i][j];
		if (match.confidence < EPS) continue;
		estimates.emplace_back(
				get_focal_from_matrix(match.homo));
	}
	int ne = estimates.size();
	if (ne < min(n - 1, 3))
		return -1;	// focal estimate fail
	sort(begin(estimates), end(estimates));
	if (ne % 2 == 1)
		return estimates[ne >> 1];
	else
		return (estimates[ne >> 1] + estimates[(ne >> 1) - 1]) * 0.5;
}


void Camera::rotation_to_angle(const Homography& r, double& rx, double& ry, double& rz) {
	using namespace Eigen;
	auto R_eigen = Map<const Eigen::Matrix<double, 3, 3, RowMajor>>(r.ptr());
	JacobiSVD<MatrixXd> svd(R_eigen, ComputeFullU | ComputeFullV);
	Matrix3d Rnew = svd.matrixU() * (svd.matrixV().transpose());
	if (Rnew.determinant() < 0)
		Rnew *= -1;

	// copy from opencv
	rx = Rnew(2,1) - Rnew(1,2);
	ry = Rnew(0,2) - Rnew(2,0);
	rz = Rnew(1,0) - Rnew(0,1);
	double c = (Rnew(0,0) + Rnew(1,1) + Rnew(2,2) - 1)*0.5;
	c = c > 1. ? 1. : c < -1. ? -1. : c;
	double theta = acos(c);
	double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
	if (s < 1e-5) {
		double t;
		if( c > 0 )
			rx = ry = rz = 0;
		else {
			t = (Rnew(0,0) + 1)*0.5; rx = sqrt(max(t,0.));
			t = (Rnew(1,1) + 1)*0.5; ry = sqrt(max(t,0.))*(Rnew(0,1) < 0 ? -1. : 1.);
			t = (Rnew(2,2) + 1)*0.5; rz = sqrt(max(t,0.))*(Rnew(0,2) < 0 ? -1. : 1.);
			if(fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (Rnew(1,2) > 0) != (ry*rz > 0) )
				rz = -rz;
			theta /= sqrt(rx*rx + ry*ry + rz*rz);
			rx *= theta;
			ry *= theta;
			rz *= theta;
		}
	} else {
		double vth = 1/(2*s);
		vth *= theta;
		rx *= vth; ry *= vth; rz *= vth;
	}
}

void Camera::angle_to_rotation(double rx, double ry, double rz, Homography& r) {
	// copy from opencv
	double theta = sqrt(rx*rx + ry*ry + rz*rz);
	if (theta < 1e-10) {
		r = Homography::I();
		return;
	}

	double c = cos(theta);
	double s = sin(theta);
	double c1 = 1. - c;
	double itheta = theta ? 1./theta : 0.;
	rx *= itheta; ry *= itheta; rz *= itheta;

	double rrt[] = { rx*rx, rx*ry, rx*rz, rx*ry, ry*ry, ry*rz, rx*rz, ry*rz, rz*rz };
	double _r_x_[] = { 0, -rz, ry, rz, 0, -rx, -ry, rx, 0 };
	r = Homography::I();
	double* R = r.ptr();

	// R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
	// where [r_x] is [0 -rz ry; rz 0 -rx; -ry rx 0]
	REP(k, 9)
		R[k] = c*R[k] + c1*rrt[k] + s*_r_x_[k];
}

void Camera::straighten(std::vector<Camera>& cameras) {
	using namespace Eigen;
	Matrix3d cov = Matrix3d::Zero();
	for (auto& c : cameras) {
		// R is from current image to reference image
		// the first column is X vector (R * [1 0 0]^T)
		const double* ptr = c.R.ptr();
		Vector3d v; v << ptr[0], ptr[3], ptr[6];
		cov += v * v.transpose();
	}
	// want to solve Cov * u == 0
	auto V = cov.jacobiSvd(ComputeFullU | ComputeFullV).matrixV();
	Vector3d normY = V.col(2);		// corrected y-vector

	Vector3d vz = Vector3d::Zero();
	for (auto& c : cameras) {
		vz(0) += c.R.ptr()[2];
		vz(1) += c.R.ptr()[5];
		vz(2) += c.R.ptr()[8];
	}
	Vector3d normX = normY.cross(vz);
	normX.normalize();
	Vector3d normZ = normX.cross(normY);

	double s = 0;
	for (auto& c : cameras) {
		Vector3d v; v << c.R.ptr()[0], c.R.ptr()[3], c.R.ptr()[6];
		s += normX.dot(v);
	}
	if (s < 0) normX *= -1, normY *= -1;	// ?

	Homography r;
	REP(i, 3) r.ptr(0)[i] = normX(i);
	REP(i, 3) r.ptr(1)[i] = normY(i);
	REP(i, 3) r.ptr(2)[i] = normZ(i);
	for (auto& c : cameras)
		c.R = r * c.R;
}

}
