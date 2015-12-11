//File: polygon.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include <utility>
#include "geometry.hh"

namespace {
inline Vec2D get_com(const std::vector<Vec2D>& poly) {
	Vec2D ret;
	for (auto& c : poly)
		ret += c;
	ret = ret * (1.0 / poly.size());
	return ret;
}
}

namespace pano {

typedef std::vector<Vec2D> Polygon;

// return n points for a n-node polygon
// will modify the order of the input
Polygon convex_hull(std::vector<Vec2D>&);

// a structure to store polygon for fast Point-In-Polygon(PIP) query
class PointInPolygon {
	public:
		PointInPolygon(const Polygon& p): poly(p) {
			m_assert(p.size() >= 3);
			com = get_com(poly);

			REP(i, p.size()) {
				float k = atan2((p[i].y - com.y), (p[i].x - com.x));
				slopes.emplace_back(k, i);
			}
			sort(slopes.begin(), slopes.end());
		}

		bool in_polygon(Vec2D p) const;

	private:
		const Polygon& poly;
		Vec2D com;	// center of mass
		std::vector<std::pair<float, int>> slopes;
};


}
