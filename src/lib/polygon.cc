//File: polygon.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "polygon.hh"
using namespace std;

namespace {
// which side is p on line(a,b)
float side(const Vec2D& a, const Vec2D& b, const Vec2D& p) {
	return (b - a).cross(p - a);
}

}

namespace pano {

Polygon convex_hull(vector<Vec2D>& pts) {
	if (pts.size() <= 3) return pts;
	m_assert(pts.size());
	sort(begin(pts), end(pts), [](const Vec2D& a, const Vec2D& b) {
			if (a.y == b.y)	return a.x < b.x;
			return a.y < b.y;
			});
	Polygon ret;
	ret.emplace_back(pts[0]);
	ret.emplace_back(pts[1]);


	// right link
	int n = pts.size();
	for (int i = 2; i < n; ++i) {
		while (ret.size() >= 2 && side(ret[ret.size() - 2], ret.back(), pts[i]) <= 0)
			ret.pop_back();
		ret.emplace_back(pts[i]);
	}

	// left link
	size_t mid = ret.size();
	ret.emplace_back(pts[n - 2]);
	for (int i = n - 3; i >= 0; --i) {
		while (ret.size() > mid && side(ret[ret.size() - 2], ret.back(), pts[i]) <= 0)
			ret.pop_back();
		ret.emplace_back(pts[i]);
	}
	return ret;
}

bool PointInPolygon::in_polygon(Vec2D p) const {
	float k = atan2((p.y - com.y), (p.x - com.x));
	auto itr = lower_bound(begin(slopes), end(slopes), make_pair(k, 0));
	int idx1, idx2;
	if (itr == slopes.end()) {
		idx1 = slopes.back().second;
		idx2 = slopes.front().second;
	} else {
		idx2 = itr->second;
		if (itr != slopes.begin())
			idx1 = (--itr)->second;
		else
			idx1 = slopes.back().second;
	}
	Vec2D p1 = poly[idx1], p2 = poly[idx2];
	// see if com, p are on the same side to line(p1,p2)
	float o1 = side(p1, p2, com), o2 = side(p1, p2, p);
	if (o1 * o2 < -EPS)
		return false;
	return true;
}

}
