// File: matrix.cc
// Date: Sun Apr 14 00:27:05 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
namespace ublas = boost::numeric::ublas;
using namespace boost::numeric::ublas;
using namespace std;

#include "matrix.hh"

template<class T>
bool InvertMatrix (const ublas::matrix<T>& input, ublas::matrix<T>& inverse) {
	typedef permutation_matrix<size_t> pmatrix;
	// create a working copy of the input
	matrix<T> A(input);
	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());
	// perform LU-factorization
	int res = lu_factorize(A,pm);
    if(res != 0)
		 return false;
	// create identity matrix of "inverse"
	inverse.assign(ublas::identity_matrix<T>(A.size1()));
	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);
	return true;
}

bool inverse(const Mat& A, Mat& Res) {
	m_assert(A.w == A.h && A.w == Res.w && Res.w == Res.h);

	matrix<real_t> in(A.w, A.h);
	for (int i = 0; i < A.w; i ++)
		for (int j = 0; j < A.h; j ++) {
			in(j, i) = A.get(i, j);
		}
	matrix<real_t> ret(A.w, A.h);
	if (!InvertMatrix(in, ret))
		return false;
	for (int i = 0; i < A.h; i ++)
		for (int j = 0; j < A.w; j ++)
			Res.get(i, j) = ret(i, j);

	return true;
}
