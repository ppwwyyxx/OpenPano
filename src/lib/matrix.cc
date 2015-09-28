// File: matrix.cc
// Date: Thu Jun 18 04:36:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <boost/numeric/mtl/mtl.hpp>
#include "matrix.hh"
#include "geometry.hh"

typedef mtl::matrix::dense2D<real_t> mtlM;
using namespace std;

ostream& operator << (std::ostream& os, const Matrix & m) {
	os << "[" << m.w << " " << m.h << "] :" << endl;
	REP(i, m.h) REP(j, m.w)
		os << m.get(i, j) << (j == m.w - 1 ? "\n" : ", ");
	return os;
}

Matrix Matrix::transpose() const {
	Matrix ret(h, w);
	REP(i, h) REP(j, w)
		ret.get(j, i) = val[i][j];
	return move(ret);
}

Matrix Matrix::prod(const Matrix & r) const {
	m_assert(w == r.h);
	const Matrix transp(r.transpose());
	Matrix ret(r.w, h);
	REP(i, h) REP(j, r.w) REP(k, w)
		ret.get(i, j) += val[i][k] * transp.get(j, k);
	return move(ret);
}

/*
 *bool Matrix::inverse(Matrix &ret) const {		// require ret to be initialized
 *    m_assert(w == h);
 *    for (int i = 0; i < h; i ++)
 *        for (int j = 0; j < w; j ++)
 *            ret.get(i, j) = get(i, j);
 *    real_t det = 1;
 *    real_t *l = new real_t[w],
 *           *m = new real_t[w];
 *    for (int k = 0; k < w; k ++) {
 *        l[k] = k, m[k] = k;
 *        real_t biga = get(k, k);
 *
 *        for (int i = k; i < w; i ++)
 *            for (int j = k; j < w; j ++) {
 *                real_t val = get(i, j);
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
 *                real_t tmp = -get(k, j);
 *                ret.get(k, j) = ret.get(i, j);
 *                ret.get(i, j) = tmp;
 *            }
 *        int j = m[k];
 *        if (j > k)
 *            for (int i = 0; i < w; i ++) {
 *                real_t tmp = -get(i, k);
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
 *                real_t tmp = ret.get(i, k);
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
 *                real_t tmp = ret.get(j, k);
 *                ret.get(j, k) = -ret.get(j, i);
 *                ret.get(j, i) = tmp;
 *            }
 *
 *        int j = m[k];
 *        if (j > k)
 *            for (int i = 0; i < w; i ++) {
*                real_t tmp = ret.get(k, i);
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
bool Matrix::inverse(Matrix &ret) const {
	m_assert(w == h && w == ret.w && ret.w == ret.h);

	mtlM input(h, w);
	REP(i, h) REP(j, w) input(i, j) = get(i, j);
	mtlM inverse(h, w);
	try {
		inv(input, inverse);
	} catch (mtl::matrix_singular) {
		cout << input << endl;
		return false;
	}

	REP(i, h) REP(j, w)
		ret.get(i, j) = inverse(i, j);

	return true;
}

bool Matrix::solve_overdetermined(Matrix & x, const Matrix & b) const {
	m_assert(h >= w);			// check overdetermined
	Matrix mt = transpose();
	Matrix mtm = mt.prod(*this);
	Matrix inverse(mtm.w, mtm.h);
	if (!mtm.inverse(inverse))		// TODO judge determinant threshold 0.001
		return false;
	x = inverse.prod(mt).prod(b);
	return true;
}

bool Matrix::SVD(Matrix& u, Matrix& s, Matrix& v) const {
	mtlM A(h, w);
	REP(i, h) REP(j, w) A(i, j) = get(i, j);
	mtlM l(h, h), m(h, w), r(w, w);
	boost::tie(l, m, r) = svd(A, 1.e-6);
	/*
	 *cout << "done" << endl;
	 *cout << l << endl << m << endl   << r << endl;
	 *mtlM result(h, w);
	 *mult(m, trans(r), result);
	 *mult(l, result, result);
	 *cout <<  result << endl;
	 */

	REP(i, h) REP(j, h)
		u.get(i, j) = l(i, j);
	REP(i, h) REP(j, w)
		s.get(i, j) = m(i, j);
	REP(i, w) REP(j, w)
		v.get(i, j) = r(i, j);

	return true;
}

void Matrix::normrot() {
	m_assert(w == 3);
	Vec p(val[0][0], val[1][0], val[2][0]);
	Vec q(val[0][1], val[1][1], val[2][1]);
	Vec r(val[0][2], val[1][2], val[2][2]);
	p.normalize();
	q.normalize();
	r.normalize();
	Vec vtmp = p.cross(q);
	real_t dist = (vtmp - r).mod();
	if (dist > 1e-6)
		r = vtmp;
	val[0][0] = p.x, val[1][0] = p.y, val[2][0] = p.z;
	val[0][1] = q.x, val[1][1] = q.y, val[2][1] = q.z;
	val[0][2] = r.x, val[1][2] = r.y, val[2][2] = r.z;
}

real_t Matrix::sqrsum() const {
	m_assert(w == 1);
	real_t sum = 0;
	REP(i, h)
		sum += sqr(val[i][0]);
	return sum;
}

Matrix Matrix::col(int i) const {
	m_assert(i < w);
	Matrix ret(1, h);
	REP(j, h)
		ret.get(j, 0) = get(j, i);
	return move(ret);
}

Matrix Matrix::I(int k) {
	Matrix ret(k, k);
	REP(i, k)
		ret.get(i, i) = 1;
	return move(ret);
}
