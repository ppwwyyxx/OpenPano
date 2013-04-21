// File: matrix.cc
// Date: Sun Apr 21 12:48:42 2013 +0800
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

bool Matrix::do_matrix_inverse(matrix<real_t>& input, matrix<real_t>& inverse) const {
	// create a permutation matrix for the LU-factorization
	permutation_matrix<size_t> pm(input.size1());
	// perform LU-factorization
	int res = lu_factorize(input, pm);
    if (res != 0) return false;
	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<real_t>(input.size1()));
	// backsubstitute to get the inverse
	lu_substitute(input, pm, inverse);
	return true;
}

bool Matrix::inverse(Matrix &ret) const {
	m_assert(w == h && w == ret.w && ret.w == ret.h);

	matrix<real_t> in(w, h);
	for (int i = 0; i < w; i ++)
		for (int j = 0; j < h; j ++) {
			in(j, i) = get(i, j);
		}

	matrix<real_t> result(w, h);
	if (!do_matrix_inverse(in, result))		// note: input will be modified
		return false;
	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++)
			ret.get(i, j) = result(i, j);

	return true;
}
