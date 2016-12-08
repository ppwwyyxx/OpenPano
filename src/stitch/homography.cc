//File: homography.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "homography.hh"

#include <Eigen/Dense>
#include <vector>

#include "lib/matrix.hh"
#include "lib/polygon.hh"
#include "match_info.hh"

using namespace std;

namespace {
inline Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
	to_eigenmap(const pano::Homography& m) {
		return Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
				(double*)m.data, 3, 3);
	}
}

namespace pano {

Homography Homography::inverse(bool* succ) const {
	using namespace Eigen;
	Homography ret;
	auto res = to_eigenmap(ret),
			 input = to_eigenmap(*this);
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
	auto m1 = to_eigenmap(*this),
			 m2 = to_eigenmap(r),
			 res = to_eigenmap(ret);
	res = m1 * m2;
	return ret;
}

std::vector<Vec2D> overlap_region(
		const Shape2D& shape1, const Shape2D& shape2,
		const Matrix& homo, const Homography& inv) {
	// use sampled edge points, rather than 4 corner, to deal with distorted homography
	// for distorted homography, the range of projected z coordinate contains 0
	// or equivalently, some point is projected to infinity
	const int NR_POINT_ON_EDGE = 100;
	Matrix edge_points(3, 4 * NR_POINT_ON_EDGE);
	float stepw = shape2.w * 1.0 / NR_POINT_ON_EDGE,
				steph = shape2.h * 1.0 / NR_POINT_ON_EDGE;
	REP(i, NR_POINT_ON_EDGE) {
		Vec2D p{-shape2.halfw() + i * stepw, -shape2.halfh()};
		edge_points.at(0, i * 4) = p.x, edge_points.at(1, i * 4) = p.y;
		p = Vec2D{-shape2.halfw() + i * stepw, shape2.halfh()};
		edge_points.at(0, i * 4 + 1) = p.x, edge_points.at(1, i * 4 + 1) = p.y;
		p = Vec2D{-shape2.halfw(), -shape2.halfh() + i * steph};
		edge_points.at(0, i * 4 + 2) = p.x, edge_points.at(1, i * 4 + 2) = p.y;
		p = Vec2D{shape2.halfw(), -shape2.halfh() + i * steph};
		edge_points.at(0, i * 4 + 3) = p.x, edge_points.at(1, i * 4 + 3) = p.y;
	}
	REP(i, 4 * NR_POINT_ON_EDGE)
		edge_points.at(2, i) = 1;
	auto transformed_pts = homo * edge_points;	//3x4n
	vector<Vec2D> pts2in1;
	REP(i, 4 * NR_POINT_ON_EDGE) {
		float denom = 1.0 / transformed_pts.at(2, i);
		Vec2D pin1{transformed_pts.at(0, i) * denom, transformed_pts.at(1, i) * denom};
		if (shape1.shifted_in(pin1))
			pts2in1.emplace_back(pin1);
	}

	// also add 4 corner of 1 to build convex hull, in case some are valid
	auto corners = shape1.shifted_corner();
	for (auto& c : corners) {
		Vec2D cin2 = inv.trans2d(c);
		if (shape2.shifted_in(cin2))
			pts2in1.emplace_back(c);
	}
	auto ret = convex_hull(pts2in1);
	return ret;
}

}
