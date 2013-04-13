// File: matrix.hh
// Date: Sun Apr 14 01:22:26 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <cstring>
#include "debugutils.hh"
#include "common.hh"

// basic 2-d array
template <typename T>
class Matrix {
	public:
		T **val;
		int w, h;

		Matrix(int m_w, int m_h):
			w(m_w), h(m_h) {
				val = new T* [h];
				for (int i = 0; i < h; i ++)
					val[i] = new T[w]();
		}

		Matrix(int m_w, int m_h, T** v)
			:w(m_w), h(m_h) {
			val = new T* [h];
			int rowlen = w * sizeof(T);

			for (int i = 0; i < h; i++) {
				val[i] = new T [w];
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
			val = new T* [h];
			for (int i = 0; i < h; i ++) {
				val[i] = new T[w]();
				memcpy(m.val[i], val[i], w * sizeof(T))	;
			}
		}

		Matrix & operator = (const Matrix & m) {
			w = m.w, h = m.h;
			val = new T* [h];
			for (int i = 0; i < h; i ++) {
				val[i] = new T[w]();
				memcpy(m.val[i], val[i], w * sizeof(T))	;
			}
		}

		T & get(int i, int j)
		{ return val[i][j]; }

		const T & get(int i, int j) const
		{ return val[i][j]; }

		template <typename TT>
		friend std::ostream& operator << (std::ostream& os, const Matrix<TT>& m);
};

template <typename T>
std::ostream& operator << (std::ostream& os, const Matrix<T>& m) {
	os << "[" << m.w << " " << m.h << "] :";
	for (int i = 0; i < m.h; i ++)
		for (int j = 0; j < m.w; j ++)
			os << m.get(i, j) << (j == m.w - 1 ? "; " : ", ");
	return os;
}

typedef Matrix<real_t> Mat;

bool inverse(const Mat& A, Mat& B);
