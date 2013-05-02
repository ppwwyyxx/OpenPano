// File: planedrawer.hh
// Date: Fri May 03 04:52:58 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#ifndef __HEAD__PLANE_DRAWER
#define __HEAD__PLANE_DRAWER
#include "color.hh"
#include "render/render.hh"

class PlaneDrawer {
	public:
		PlaneDrawer(RenderBase* m_r):
			 render(m_r) {}

		virtual ~PlaneDrawer(){}

		void set_color(Color m_c)
		{ c = m_c; }

		inline void point(int x, int y)
		{ render->write(x, y, c); }

		inline void point(Coor v)
		{ render->write(v.x, v.y, c); }


		inline void line(Coor s, Coor t)
		{ Bresenham(s, t); }

		inline void line(Vec2D s, Vec2D t)
		{ line(Coor(s.x, s.y), Coor(t.x, t.y)); }

		inline void line(Line2D l)
		{ line(l.first, l.second); }

		inline void line(std::pair<Vec2D, Vec2D> l)
		{ line(l.first, l.second); }

		void circle(Coor o, int r);

		void cross(Coor o, int r);

		void arrow(Coor o, real_t dir, int r);

		void polygon(Polygon p) {
			for (unsigned int i = 0; i < p.size() - 1; i++)
				line(p[i], p[i + 1]);
			line(p.back(), p.front());
		}

		void polygon(std::vector<Vec2D> p)
		{ polygon(to_renderable(p)); }

		void finish()
		{ render->finish(); }

	protected:
		void Bresenham(Coor s, Coor t);
		RenderBase* render;
		Color c = Color::BLACK;

		Polygon to_renderable(std::vector<Vec2D> p) {
			Polygon ret;
			for (auto v : p)
				ret.push_back(Coor(v.x, v.y));
			return ret;
		}
};

#endif //HEAD
