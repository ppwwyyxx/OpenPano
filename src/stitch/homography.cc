//File: homography.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "homography.hh"
#include <Eigen/Dense>
using namespace Eigen;

#define EIGENMAP_FROM_HOMO(homo, var) \
	auto var = Map<Eigen::Matrix<double, 3, 3, RowMajor>>((double*)(homo).data, 3, 3);

namespace pano {

Homography Homography::inverse(bool* succ) const {
	Homography ret;
	EIGENMAP_FROM_HOMO(ret, res);
	EIGENMAP_FROM_HOMO(*this, input);
	FullPivLU<Eigen::Matrix<double,3,3,RowMajor>> lu(input);
	if (succ == nullptr) {
		m_assert(lu.isInvertible());
	} else {
		*succ = lu.isInvertible();
		if (! *succ) return ret;
	}
	res = lu.inverse().eval();
	return ret;
}

Homography Homography::operator * (const Homography& r) const {
	Homography ret;
	EIGENMAP_FROM_HOMO(*this, m1);
	EIGENMAP_FROM_HOMO(r, m2);
	EIGENMAP_FROM_HOMO(ret, res);
	res = m1 * m2;
	return ret;
}

}
