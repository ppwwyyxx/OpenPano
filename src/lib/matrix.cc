// File: matrix.cc
// Date: Thu Jun 18 04:36:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <boost/numeric/mtl/mtl.hpp>
#include "matrix.hh"
#include "geometry.hh"
#include "timer.hh"

typedef mtl::matrix::dense2D<double> mtlM;
using namespace std;

namespace {
	inline mtlM matrix_to_mtl(const Matrix& m) {
		mtlM ret(m.rows(), m.cols());
		REP(i, m.rows()) REP(j, m.cols())
			ret(i, j) = m.at(i, j);
		return ret;
	}

	inline Matrix mtl_to_matrix(const mtlM& m) {
		int r = m.num_rows(), c = m.num_cols();
		Matrix ret(r, c);
		REP(i, r) REP(j, c)
			ret.at(i, j) = m(i, j);
		return ret;
	}
}

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

/*
 *bool Matrix::inverse(Matrix &ret) const {		// require ret to be initialized
 *    m_assert(w == h);
 *    for (int i = 0; i < h; i ++)
 *        for (int j = 0; j < w; j ++)
 *            ret.get(i, j) = get(i, j);
 *    double det = 1;
 *    double *l = new double[w],
 *           *m = new double[w];
 *    for (int k = 0; k < w; k ++) {
 *        l[k] = k, m[k] = k;
 *        double biga = get(k, k);
 *
 *        for (int i = k; i < w; i ++)
 *            for (int j = k; j < w; j ++) {
 *                double val = get(i, j);
 *                if (fabs(val) > fabs(biga)) {
 *                    biga = val;
 *                    l[k] = i, m[k] = j;
 *                }
 *            }
 *
 *        if (biga == 0) return 0;
 *
 *        int i = l[k];
 *        if (i > k)
 *            for (int j = 0; j < w; j ++) {
 *                double tmp = -get(k, j);
 *                ret.get(k, j) = ret.get(i, j);
 *                ret.get(i, j) = tmp;
 *            }
 *        int j = m[k];
 *        if (j > k)
 *            for (int i = 0; i < w; i ++) {
 *                double tmp = -get(i, k);
 *                ret.get(i, k) = ret.get(i, j);
 *                ret.get(i, j) = tmp;
 *            }
 *
 *        for (int i = 0; i < w; i ++)
 *            if (i != k)
 *                ret.get(i, k) /= -biga;
 *
 *        for (int i = 0; i < w; i ++)
 *            if (i != k) {
 *                double tmp = ret.get(i, k);
 *                for (int j = 0; j < w; j ++)
 *                    if (j != k)
 *                        ret.get(i, j) += tmp * ret.get(k, j);
 *            }
 *
 *        for (int j = 0; j < w; j ++)
 *            if (j != k)
 *                ret.get(k, j) /= biga;
 *
 *        det *= biga;
 *        ret.get(k, k) = 1.0 / biga;
 *    }
 *
 *    for (int k = w - 1; k >= 0; k --) {
 *        int i = l[k];
 *        if (i > k)
 *            for (int j = 0; j < w; j ++) {
 *                double tmp = ret.get(j, k);
 *                ret.get(j, k) = -ret.get(j, i);
 *                ret.get(j, i) = tmp;
 *            }
 *
 *        int j = m[k];
 *        if (j > k)
 *            for (int i = 0; i < w; i ++) {
*                double tmp = ret.get(k, i);
*                ret.get(k, i) = -ret.get(j, i);
*                ret.get(j, i) = tmp;
*            }
*    }
*
*    delete[] l;
*    delete[] m;
*    cout << prod(ret) << endl;
*    return det;
*}
*/

// TODO speedup? better algo?
bool Matrix::inverse(Matrix &ret) const {
	m_assert(m_rows == m_cols);
	int n = m_rows;
	mtlM input(matrix_to_mtl(*this));
	mtlM inverse(n, n);
	try {
		inv(input, inverse);
	} catch (mtl::matrix_singular) {
		//cout << input << endl;
		return false;
	}

	ret = mtl_to_matrix(inverse);
	return true;
}

// pseudo inverse by SVD
Matrix Matrix::pseudo_inverse() const {
	m_assert(m_rows >= m_cols);
	mtlM A(matrix_to_mtl(*this));
	mtlM uu, ss, vv;
	boost::tie(uu, ss, vv) = svd(A, 1.e-6);
	REP(i, m_cols) {
		if (ss(i, i) > EPS)
			ss(i, i) = 1 / ss(i, i);
		else
			ss(i, i) = 0;
	}
	return mtl_to_matrix(mtlM(vv * trans(ss) * trans(uu)));
}

Matrix Matrix::solve_overdetermined(const Matrix & b) const {
	m_assert(m_rows >= m_cols);			// check overdetermined
	Matrix mt = transpose();
	Matrix mtm = mt.prod(*this);
	Matrix inverse;
	if (!mtm.inverse(inverse)) {
		inverse = pseudo_inverse();
		return inverse.prod(b);
	}
	return inverse.prod(mt).prod(b);
}

bool Matrix::SVD(Matrix& u, Matrix& s, Matrix& v) const {
	mtlM A(matrix_to_mtl(*this));
	mtlM uu(m_rows, m_rows), ss(m_rows, m_cols), vv(m_cols, m_cols);
	boost::tie(uu, ss, vv) = svd(A, 1.e-6);
	/*
	 *cout << "done" << endl;
	 *cout << l << endl << m << endl   << r << endl;
	 *mtlM result(h, w);
	 *mult(m, trans(r), result);
	 *mult(l, result, result);
	 *cout <<  result << endl;
	 */

	u = Matrix(m_rows, m_rows);
	REP(i, m_rows) REP(j, m_rows)
		u.at(i, j) = uu(i, j);
	s = Matrix(m_rows, m_cols);
	REP(i, m_rows) REP(j, m_cols)
		s.at(i, j) = ss(i, j);
	v = Matrix(m_cols, m_cols);
	REP(i, m_cols) REP(j, m_cols)
		v.at(i, j) = vv(i, j);
	return true;
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
