// File: matrix.hh
// Date: Sun Apr 21 12:48:40 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <cstring>
#include <boost/numeric/ublas/matrix.hpp>
#include "debugutils.hh"
#include "common.hh"

// basic 2-d array
class Matrix {
	public:
		real_t **val;
		int w, h;

		Matrix(int m_w, int m_h):
			w(m_w), h(m_h) {
				val = new real_t* [h];
				for (int i = 0; i < h; i ++)
					val[i] = new real_t[w]();
		}

		Matrix(int m_w, int m_h, real_t** v)
			:w(m_w), h(m_h) {
			val = new real_t* [h];
			int rowlen = w * sizeof(real_t);

			for (int i = 0; i < h; i++) {
				val[i] = new real_t [w];
				if (v)
					memcpy(val[i], v[i], rowlen);
			}
		}

		~Matrix() {
			for (int i = 0; i < h; i++)
				delete [] val[i];
			delete [] val;
		}

		Matrix(const Matrix& m) {
			w = m.w, h = m.h;
			val = new real_t* [h];
			for (int i = 0; i < h; i ++) {
				val[i] = new real_t[w]();
				memcpy(m.val[i], val[i], w * sizeof(real_t));
			}
		}

		Matrix & operator = (const Matrix & m) {
			w = m.w, h = m.h;
			val = new real_t* [h];
			for (int i = 0; i < h; i ++) {
				val[i] = new real_t[w]();
				memcpy(m.val[i], val[i], w * sizeof(real_t));
			}
			return *this;
		}

		real_t & get(int i, int j)
		{ return val[i][j]; }

		const real_t & get(int i, int j) const
		{ return val[i][j]; }

		bool inverse(Matrix & ret) const;

		friend std::ostream& operator << (std::ostream& os, const Matrix & m);

	private:
		bool do_matrix_inverse(boost::numeric::ublas::matrix<real_t>& input,
				boost::numeric::ublas::matrix<real_t>& inverse) const;
};

