// File: matrix.cc
// Date: Thu Jun 18 04:36:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "matrix.hh"

#include <algorithm>
#include <memory>
#include <Eigen/Dense>

#include "geometry.hh"
#include "timer.hh"

using namespace std;

#define EIGENMAP_FROM_MATRIX(m, var) \
	auto var = Map<Eigen::Matrix<double, Dynamic, Dynamic, RowMajor>>((double*)(m).ptr(), (m).rows(), (m).cols());

//TODO speedup at() to avoid channel mult

ostream& operator << (std::ostream& os, const Matrix & m) {
	os << "[" << m.rows() << " " << m.cols() << "] :" << endl;
	REP(i, m.rows()) REP(j, m.cols())
		os << m.at(i, j) << (j == m.cols() - 1 ? "\n" : ", ");
	return os;
}

Matrix Matrix::transpose() const {
	Matrix ret(m_cols, m_rows);
	REP(i, m_rows) REP(j, m_cols)
		ret.at(j, i) = at(i, j);
	return move(ret);
}

Matrix Matrix::prod(const Matrix & r) const {
	using namespace Eigen;
	Matrix ret(m_rows, r.cols());
	EIGENMAP_FROM_MATRIX(*this, m1);
	EIGENMAP_FROM_MATRIX(r, m2);
	EIGENMAP_FROM_MATRIX(ret, res);
	res = m1 * m2;
	return move(ret);
}

Matrix Matrix::elem_prod(const Matrix& r) const {
	m_assert(m_rows == r.rows() && m_cols == r.cols());
	Matrix ret(m_rows, m_cols);
	double* res = ret.ptr();
	const double *rl = ptr(), *rr = r.ptr();
	REP(i, pixels()) res[i] = rl[i] * rr[i];
	return ret;
}

Matrix Matrix::operator - (const Matrix& r) const {
	m_assert(rows() == r.rows() && cols() == r.cols());
	Matrix ret(rows(), cols());
	double* res = ret.ptr();
	const double *rl = ptr(), *rr = r.ptr();
	REP(i, pixels()) res[i] = rl[i] - rr[i];
	return ret;
}
Matrix Matrix::operator + (const Matrix& r) const {
	m_assert(rows() == r.rows() && cols() == r.cols());
	Matrix ret(rows(), cols());
	double* res = ret.ptr();
	const double *rl = ptr(), *rr = r.ptr();
	REP(i, pixels()) res[i] = rl[i] + rr[i];
	return ret;
}

bool Matrix::inverse(Matrix &ret) const {
	m_assert(m_rows == m_cols);
	using namespace Eigen;
	ret = Matrix(m_rows, m_rows);
	EIGENMAP_FROM_MATRIX(*this, input);
	EIGENMAP_FROM_MATRIX(ret, res);
	FullPivLU<Eigen::Matrix<double,Dynamic,Dynamic,RowMajor>> lu(input);
	if (! lu.isInvertible()) return false;
	res = lu.inverse().eval();
	return true;
}

// pseudo inverse by SVD
Matrix Matrix::pseudo_inverse() const {
	using namespace Eigen;
	m_assert(m_rows >= m_cols);
	EIGENMAP_FROM_MATRIX(*this, input);
	JacobiSVD<MatrixXd> svd(input, ComputeThinU | ComputeThinV);
	auto sinv = svd.singularValues();
	REP(i, m_cols) {
		if (sinv(i) > EPS)
			sinv(i) = 1.0 / sinv(i);
		else
			sinv(i) = 0;
	}
	Matrix ret(m_cols, m_rows);
	EIGENMAP_FROM_MATRIX(ret, res);
	res = svd.matrixV() * sinv.asDiagonal() * svd.matrixU().transpose();
	m_assert(ret.at(0, 2) == res(0, 2));
	return ret;
}

void Matrix::normrot() {
	m_assert(m_cols == 3);
	Vec p(at(0, 0), at(1, 0), at(2, 0));
	Vec q(at(0, 1), at(1, 1), at(2, 1));
	Vec r(at(0, 2), at(1, 2), at(2, 2));
	p.normalize();
	q.normalize();
	r.normalize();
	Vec vtmp = p.cross(q);
	double dist = (vtmp - r).mod();
	if (dist > 1e-6)
		r = vtmp;
	at(0, 0) = p.x, at(1, 0) = p.y, at(2, 0) = p.z;
	at(0, 1) = q.x, at(1, 1) = q.y, at(2, 1) = q.z;
	at(0, 2) = r.x, at(1, 2) = r.y, at(2, 2) = r.z;
}

double Matrix::sqrsum() const {
	m_assert(m_cols == 1);
	double sum = 0;
	REP(i, m_rows)
		sum += sqr(at(i, 0));
	return sum;
}

Matrix Matrix::col(int i) const {
	m_assert(i < m_cols);
	Matrix ret(m_rows, 1);
	REP(j, m_rows)
		ret.at(j, 0) = at(j, i);
	return move(ret);
}

Matrix Matrix::I(int k) {
	Matrix ret(k, k);
	ret.zero();
	REP(i, k)
		ret.at(i, i) = 1;
	return move(ret);
}

void Matrix::zero() {
	double* p = ptr();
	int n = pixels();
	memset(p, 0, n * sizeof(double));
}
