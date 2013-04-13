// File: matrixrender.hh
// Date: Fri Apr 05 17:12:28 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "color.hh"
#include "matrix.hh"
#include "render.hh"

class MatrixRender: public RenderBase {
	public:
		Matrix<Color>* mat;

		MatrixRender(const Geometry &m_g):
			RenderBase(m_g)
		{ mat = new Matrix<Color>(m_g.w, m_g.h); }

		~MatrixRender()
		{ delete mat; }

		void _write(int x, int y, const Color &c)
		{ (mat->val)[y][x] = c; }

		Color& get(int x, int y)
		{ return mat->val[y][x]; }
};

