//File: brief.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "brief.hh"
#include <random>

#include "lib/debugutils.hh"
#include "lib/timer.hh"

using namespace std;

namespace pano {

BRIEF::BRIEF(const Mat32f& img, const vector<SSPoint>& points,
		const BriefPattern& pattern):
	img(img), points(points), pattern(pattern) { }

vector<Descriptor> BRIEF::get_descriptor() const {
	TotalTimer tm("brief descriptor");
	vector<Descriptor> ret;
	const int half = pattern.s / 2;
	for (auto& p : points) {
		int x = round(p.real_coor.x * img.width()),
				y = round(p.real_coor.y * img.height());
		if (x >= half && x + half < img.width() && y >= half && y + half < img.height()) {
			auto desp = calc_descriptor(p);
			ret.emplace_back(move(desp));
		}
	}
	return ret;
}

Descriptor BRIEF::calc_descriptor(const SSPoint& p) const {
	int x = round(p.real_coor.x * img.width()),
			y = round(p.real_coor.y * img.height());
	const int n = pattern.pattern.size();
	const int half = pattern.s / 2;
	vector<bool> bits(n, false);
	auto pixel = [&](int r, int c) {
		const float* ptr = img.ptr(r, c);
		return (ptr[0] + ptr[1] + ptr[2]) / 3;
	};
	REP(i, n) {
		int p1 = pattern.pattern[i].first,
				p2 = pattern.pattern[i].second;
		int y1 = y + p1 / pattern.s - half,
				x1 = x + p1 % pattern.s - half;
		int y2 = y + p2 / pattern.s - half,
				x2 = x + p2 % pattern.s - half;
		bits[i] = pixel(y1, x1) > pixel(y2, x2);
	}
	Descriptor ret;
	ret.coor = p.real_coor;
	ret.descriptor.resize(n / 32, 0);
	REP(i, n) if (bits[i]) {
		int idx = i / 32,
				offset = i % 32;
		int* ptr = (int*)&ret.descriptor[idx];
		*ptr = *ptr | (1 << offset);
	}
	return ret;
}


// implement pattern II in BRIEF orignal paper
BriefPattern BRIEF::gen_brief_pattern(int s, int n) {
	m_assert(s % 2 == 1);
	m_assert(n % 32 == 0);
	mt19937 randgen{std::random_device()()};
	normal_distribution<> d(0.5 * s, 0.2 * s);

	BriefPattern ret;
	ret.s = s;
	auto get_sample = [&]() {
		int ret = -1;
		while (ret < 0 || ret >= s)
			ret = round(d(randgen));
		return ret;
	};
	while (n--) {
		int x1 = get_sample();
		int y1 = get_sample();
		int x2, y2;
		do {
			x2 = get_sample();
			y2 = get_sample();
		} while (y1 == x1 && y2 == x2);
		ret.pattern.emplace_back(y1 * s + x1, y2 * s + x2);
	}
	return ret;
}


}
/*
 *
 *#include <iostream>
 *int main() {
 *  auto p = pano::gen_brief_pattern(9, 256);
 *  for (auto& pair: p.pattern) {
 *    cout << pair.first/9 << " " << pair.first % 9
 *      << "->" << pair.second/ 9 << " " << pair.second %9<< endl;
 *  }
 *}
 */
