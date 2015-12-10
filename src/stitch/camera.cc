//File: camera.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "camera.hh"

#include <Eigen/Dense>
#include <algorithm>
#include <string>

#include "lib/timer.hh"
#include "match_info.hh"
#include "homography.hh"
using namespace std;
using namespace pano;
// Implement stuffs about camera K,R matrices
namespace {

// See: Creating Full View Panoramic Image Mosaics - Szeliski
double get_focal_from_matrix(const Homography& h) {
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

namespace pano {

Camera::Camera() : R(Homography::I()) { }

Homography Camera::K() const {
	Homography ret{Homography::I()};
	ret[0] = focal;
	ret[2] = ppx;
	ret[4] = focal * aspect;
	ret[5] = ppy;
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


//https://en.wikipedia.org/wiki/Rotation_matrix?oldformat=true#Determining_the_axis
void Camera::rotation_to_angle(const Homography& r, double& rx, double& ry, double& rz) {
	using namespace Eigen;
	auto R_eigen = Map<const Eigen::Matrix<double, 3, 3, RowMajor>>(r.data);

	JacobiSVD<MatrixXd> svd(R_eigen, ComputeFullU | ComputeFullV);
	Matrix3d Rnew = svd.matrixU() * (svd.matrixV().transpose());
	if (Rnew.determinant() < 0)
		Rnew *= -1;

	// r is eigenvector of R with eigenvalue=1
	rx = Rnew(2,1) - Rnew(1,2);
	ry = Rnew(0,2) - Rnew(2,0);
	rz = Rnew(1,0) - Rnew(0,1);

	double s = sqrt(rx*rx + ry*ry + rz*rz);
	if (s < GEO_EPS) {
		rx = ry = rz = 0;
	} else {
		// 1 + 2 * cos(theta) = trace(R)
		double cos = (Rnew(0,0) + Rnew(1,1) + Rnew(2,2) - 1) * 0.5;
		cos = cos > 1. ? 1. : cos < -1. ? -1. : cos;		// clip
		double theta = acos(cos);

		double mul = 1.0 / s * theta;
		rx *= mul; ry *= mul; rz *= mul;
	}
}

//https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
void Camera::angle_to_rotation(double rx, double ry, double rz, Homography& r) {
	double theta = rx*rx + ry*ry + rz*rz;
	if (theta < GEO_EPS_SQR) {	// theta ^2
		// first order Taylor. see code of ceres-solver
		r = Homography{{1, -rz, ry, rz, 1, -rx, -ry, rx, 1}};
		return;
	}
	theta = sqrt(theta);
	double itheta = theta ? 1./theta : 0.;
	rx *= itheta; ry *= itheta; rz *= itheta;

	// outer product with itself
	double u_outp[] = {rx*rx, rx*ry, rx*rz, rx*ry, ry*ry, ry*rz, rx*rz, ry*rz, rz*rz };
	// cross product matrix
	double u_crossp[] = {0, -rz, ry, rz, 0, -rx, -ry, rx, 0 };

	r = Homography::I();

	double c = cos(theta),
				 s = sin(theta),
				 c1 = 1 - c;
	r.mult(c);
	REP(k, 9)
		r[k] += c1 * u_outp[k] + s * u_crossp[k];
}

void Camera::straighten(std::vector<Camera>& cameras) {
	using namespace Eigen;
	Matrix3d cov = Matrix3d::Zero();
	for (auto& c : cameras) {
		// R is from reference image to current image
		// the first row is X vector ([1,0,0] * R)
		Vector3d v;
		v << c.R[0], c.R[1], c.R[2];
		cov += v * v.transpose();
	}
	// want to solve Cov * u == 0
	auto V = cov.jacobiSvd(ComputeFullU | ComputeFullV).matrixV();
	Vector3d normY = V.col(2);		// corrected y-vector

	Vector3d vz = Vector3d::Zero();
	for (auto& c : cameras) {
		vz(0) += c.R[6];
		vz(1) += c.R[7];
		vz(2) += c.R[8];
	}
	Vector3d normX = normY.cross(vz);
	normX.normalize();
	Vector3d normZ = normX.cross(normY);

	double s = 0;
	for (auto& c : cameras) {
		Vector3d v; v << c.R[0], c.R[1], c.R[2];
		s += normX.dot(v);
	}
	if (s < 0) normX *= -1, normY *= -1;	// ?

	Homography r;
	REP(i, 3) r[i * 3] = normX(i);
	REP(i, 3) r[i * 3 + 1] = normY(i);
	REP(i, 3) r[i * 3 + 2] = normZ(i);
	for (auto& c : cameras)
		c.R = c.R * r;
}

}
