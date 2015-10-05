// File: matrix.hh
// Date: Sat May 04 01:32:05 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <cstring>
#include <memory>
#include "mat.h"
#include "debugutils.hh"
#include "common.hh"

class Matrix : public Mat<double> {
	public:
		Matrix(){}

		Matrix(int rows, int cols):
			Mat<double>(rows, cols, 1) {}

		Matrix(const Mat<double>& r):
			Mat<double>(r) {}

		bool inverse(Matrix & ret) const;

		Matrix pseudo_inverse() const;

		Matrix transpose() const;

		Matrix prod(const Matrix & r) const;

		bool SVD(Matrix & u, Matrix & s, Matrix & v) const;

		void normrot();

		double sqrsum() const;

		Matrix col(int i) const;

		void zero();

		static Matrix I(int);

		friend std::ostream& operator << (std::ostream& os, const Matrix & m);

};

