// File: matrix.cc
// Date: Thu Jun 18 04:36:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <eigen3/Eigen/Dense>
#include "matrix.hh"
#include "geometry.hh"
#include "timer.hh"

using namespace std;

//TODO speedup at()

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
	m_assert(m_cols == r.rows());
	const Matrix transp(r.transpose());
	Matrix ret(m_rows, r.cols());
	ret.zero();

	REP(i, m_rows) REP(j, r.cols()) REP(k, m_cols)
		ret.at(i, j) += at(i, k) * transp.at(j, k);
	return move(ret);
}

bool Matrix::inverse(Matrix &ret) const {
	m_assert(m_rows == m_cols);
	using namespace Eigen;
	ret = Matrix(m_rows, m_rows);
	auto res = Map<Eigen::Matrix<double, Dynamic, Dynamic, RowMajor>>(ret.ptr(), m_rows, m_cols);
	auto input = Map<Eigen::Matrix<double, Dynamic, Dynamic, RowMajor>>(m_data.get(), m_rows, m_cols);
	FullPivLU<Eigen::Matrix<double,Dynamic,Dynamic,RowMajor>> lu(input);
	if (not lu.isInvertible()) return false;
	res = lu.inverse().eval();
	return true;
}

// pseudo inverse by SVD
Matrix Matrix::pseudo_inverse() const {
	using namespace Eigen;
	m_assert(m_rows >= m_cols);
	auto input = Map<Eigen::Matrix<double, Dynamic, Dynamic, RowMajor>>(m_data.get(), m_rows, m_cols);
	JacobiSVD<MatrixXd> svd(input, ComputeThinU | ComputeThinV);
	auto sinv = svd.singularValues();
	REP(i, m_cols) {
		if (sinv(i) > EPS)
			sinv(i) = 1.0 / sinv(i);
		else
			sinv(i) = 0;
	}
	Matrix ret(m_cols, m_rows);
	auto res = Map<Eigen::Matrix<double, Dynamic, Dynamic, RowMajor>>(ret.ptr(), m_cols, m_rows);
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
	//REP(i, n) p[i] = 0;
	memset(p, 0, n * sizeof(double));
}
