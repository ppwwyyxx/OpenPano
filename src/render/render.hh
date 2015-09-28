// File: render.hh
// Date: Sun Dec 29 03:21:47 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#pragma once

#include <memory>
#include "lib/mat.h"
#include "lib/matrix.hh"
#include "lib/color.hh"
#include "lib/common.hh"

class RenderBase {
	public:
		RenderBase(const Geometry &m_g):
			geo(m_g){}

		virtual ~RenderBase(){};

		virtual void init() {}
		// execute before write

		virtual void finish() {}
		// execute after write

		void write(int x, int y, const Color &c) {
			if (x < 0 || x >= geo.w || y < 0 || y >= geo.h)
				return;

			_write(x, y, c);
			render_cnt ++;
		}

		void write(const Mat32f& r) {
			REP(i, geo.h) {
				REP(j, geo.w) {
					const float* p = r.ptr(i, j);
					write(j, i, Color(p[0], p[1], p[2]));
				}
			}
		}

		const Geometry& get_geo() const
		{ return geo; }

		int get_cnt() const
		{ return render_cnt; }

	private:
		int render_cnt = 0;

	protected:
		Geometry geo;
		virtual void _write(int x, int y, const Color &c) = 0;

};


