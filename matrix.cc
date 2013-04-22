// File: matrix.cc
// Date: Mon Apr 22 21:05:08 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <boost/numeric/ublas/lu.hpp>
#include "matrix.hh"

using namespace boost::numeric::ublas;
using namespace std;

ostream& operator << (std::ostream& os, const Matrix & m) {
	os << "[" << m.w << " " << m.h << "] :" << endl;
	for (int i = 0; i < m.h; i ++)
		for (int j = 0; j < m.w; j ++)
			os << m.get(i, j) << (j == m.w - 1 ? "\n" : ", ");
	return os;
}

Matrix Matrix::transpose() const {
	Matrix ret(h, w);
	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++)
			ret.get(j, i) = val[i][j];
	return move(ret);
}

Matrix Matrix::prod(const Matrix & r) const {
	m_assert(w == r.h);
	const Matrix transp(r.transpose());
	Matrix ret(r.w, h);
	for (int i = 0; i < h; i ++)
		for (int j = 0; j < r.w; j ++)
			for (int k = 0; k < w; k ++)
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

	matrix<real_t> input(w, h);
	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++) {
			input(i, j) = get(i, j);
		}

	matrix<real_t> inverse(w, h);
	// create a permutation matrix for the LU-factorization
	permutation_matrix<size_t> pm(input.size1());
	// perform LU-factorization
	int res = lu_factorize(input, pm);
	if (res != 0) return false;
	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<real_t>(input.size1()));
	// backsubstitute to get the inverse
	lu_substitute(input, pm, inverse);

	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++)
			ret.get(i, j) = inverse(i, j);

	/*
	 *cout << *this << endl;
	 *cout << ret << endl;
	 */
	cout << prod(ret) << endl;
	return true;
}

bool Matrix::solve_overdetermined(Matrix & x, const Matrix & b) const {
	m_assert(h >= w);			// check overdetermined
	Matrix mt = transpose();
	Matrix mtm = mt.prod(*this);
	Matrix inverse(mtm.w, mtm.h);
	if (!mtm.inverse(inverse))		// TODO judge determinant threshold 0.001
		return false;
	x = move(inverse.prod(mt).prod(b));
	return true;
}
