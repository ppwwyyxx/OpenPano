// File: stitcher.cc
// Date: Tue Apr 23 11:27:13 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>
//
#include "stitcher.hh"
#include "transformer.hh"
using namespace std;

shared_ptr<Img> Stitcher::stitch() const {
	Vec2D corner[4] = {Vec2D(0, 0), Vec2D(img2->w, 0), Vec2D(0, img2->h), Vec2D(img2->w, img2->h)};
	Coor min(0, 0), max(img1->w, img1->h);
	for (auto &i : corner) {
		Vec2D newcorner = TransFormer::cal_project(trans, i);
		min.update_min(Coor(floor(newcorner.x), floor(newcorner.y)));
		max.update_max(Coor(ceil(newcorner.x), ceil(newcorner.y)));
	}

	Coor newsize = max - min;
	Vec2D offset = Vec2D(-min.x, -min.y);

	bool **filled = new bool*[newsize.y];
	REP(i, newsize.y) filled[i] = new bool[newsize.x]();

	cout << newsize << endl;
	shared_ptr<Img> ret(new Img(newsize.x, newsize.y));
	ret->fill(Color::BLACK);

	REPL(i, 1, ret->h - 1) REPL(j, 1, ret->w - 1) {
		Vec2D black = Vec2D(j, i) - offset;
		Vec2D old = TransFormer::cal_project(inv, black);
		if (between(old.x, 0, img2->w) && between(old.y, 0, img2->h))
			ret->set_pixel(i, j, img2->get_pixel(old.y, old.x));
	}

	REP(i, img1->h)
		REP(j, img1->w) {
			Vec2D newcoor = Vec2D(j, i) + offset;
			Color col;
			if (filled[(int)round(newcoor.y)][(int)round(newcoor.x)])
				col = (ret->get_pixel(newcoor) + img1->get_pixel(i, j)) * 0.5;
			else
				col = img1->get_pixel(i, j);
			ret->set_pixel(newcoor.y, newcoor.x, col);
		}

	free_2d<bool>(filled, newsize.y);
	return ret;
}
