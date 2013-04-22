// File: matrix.hh
// Date: Tue Apr 23 00:21:57 2013 +0800
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

		Matrix(int m_w, int m_h):		// initialize with value 0
			w(m_w), h(m_h) {
				val = new real_t* [h];
				REP(i, h)
					val[i] = new real_t[w]();
			}

		~Matrix() { free_2d<real_t>(val, h); }

		// something bad
		Matrix(const Matrix& m) {
			w = m.w, h = m.h;
			val = new real_t* [h];
			REP(i, h) {
				val[i] = new real_t[w]();
				memcpy(m.val[i], val[i], w * sizeof(real_t));
			}
		}

		Matrix & operator = (const Matrix & m) {
			Matrix tmp(m);
			*this = std::move(tmp);
			return *this;
		}

		Matrix & operator = (Matrix && r) {
			m_assert(this != &r);
			free_2d<real_t>(val, h);
			val = r.val;
			w = r.w, h = r.h;
			r.val = nullptr;
			return *this;
		}

		Matrix(Matrix&& r) {
			val = r.val;
			w = r.w, h = r.h;
			r.val = nullptr;
		}
		/*
		 *Matrix(Matrix && r):val(nullptr) { *this = std::move(r); }
		 */
		// something bad

		real_t & get(int i, int j)
		{ return val[i][j]; }

		const real_t & get(int i, int j) const
		{ return val[i][j]; }

		bool inverse(Matrix & ret) const;

		Matrix transpose() const;

		Matrix prod(const Matrix & r) const;

		bool solve_overdetermined(Matrix & x, const Matrix & b) const;		//


		friend std::ostream& operator << (std::ostream& os, const Matrix & m);

};

