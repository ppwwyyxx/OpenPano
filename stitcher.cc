// File: stitcher.cc
// Date: Tue Apr 23 00:19:09 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>
//
#include "stitcher.hh"
#include "transformer.hh"
using namespace std;

shared_ptr<Img> Stitcher::stitch() const {
	Coor corner[4] = {Coor(0, 0), Coor(img2->w, 0), Coor(0, img2->h), Coor(img2->w, img2->h)};
	Coor min(0, 0), max(img1->w, img1->h);
	for (auto &i : corner) {
		Vec2D newcorner = TransFormer::cal_project(trans, i);
		min.update_min(Coor(floor(newcorner.x), floor(newcorner.y)));
		max.update_max(Coor(ceil(newcorner.x), ceil(newcorner.y)));
	}

	Coor newsize = max - min;
	Coor offset = min * (-1);

	bool **filled = new bool*[newsize.y];
	REP(i, newsize.y) filled[i] = new bool[newsize.x]();

	cout << newsize << endl;
	shared_ptr<Img> ret(new Img(newsize.x, newsize.y));
	ret->fill(Color::BLACK);

	REP(i, img2->h) REP(j, img2->w) {
		Color col = img2->get_pixel(i, j);
		Vec2D transformed = TransFormer::cal_project(trans, Coor(j, i));
		Coor newcoor = Coor(round(transformed.x), round(transformed.y)) + offset;
		ret->set_pixel(newcoor.y, newcoor.x, col);
		filled[newcoor.y][newcoor.x] = true;
	}

	REPL(i, 1, ret->h - 1) REPL(j, 1, ret->w - 1)
		if (!filled[i][j]) {
			Coor black = Coor(j, i) - offset;
			Vec2D old = TransFormer::cal_project(inv, black);
			/*
			 *Vec2D check = TransFormer::cal_project(trans, Coor(old.x, old.y));
			 *cout << black << endl;
			 *cout << check << endl;
			 */
			if (between(old.x, 0, img2->w) && between(old.y, 0, img2->h))
				ret->set_pixel(i, j, img2->get_pixel(old.y, old.x));
		}

	/*
	 *REP(i, img1->h)
	 *    REP(j, img1->w) {
	 *        Coor newcoor = Coor(j, i) + offset;
	 *        Color col;
	 *        if (filled[newcoor.y][newcoor.x])
	 *            col = (ret->get_pixel(newcoor) + img1->get_pixel(i, j)) * 0.5;
	 *        else
	 *            col = img1->get_pixel(i, j);
	 *        ret->set_pixel(newcoor.y, newcoor.x, col);
	 *        filled[newcoor.y][newcoor.x] = true;
	 *    }
	 */

	free_2d<bool>(filled, newsize.y);
	return ret;
}
