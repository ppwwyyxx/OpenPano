// File: matrix.cc
// Date: Sun Apr 21 21:26:42 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include <boost/numeric/ublas/lu.hpp>
#include "matrix.hh"

using namespace boost::numeric::ublas;
using namespace std;

ostream& operator << (std::ostream& os, const Matrix & m) {
	os << "[" << m.w << " " << m.h << "] :";
	for (int i = 0; i < m.h; i ++)
		for (int j = 0; j < m.w; j ++)
			os << m.get(i, j) << (j == m.w - 1 ? "; " : ", ");
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

bool Matrix::inverse(Matrix &ret) const {
	m_assert(w == h && w == ret.w && ret.w == ret.h);

	matrix<real_t> input(w, h);
	for (int i = 0; i < w; i ++)
		for (int j = 0; j < h; j ++) {
			input(j, i) = get(i, j);
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
