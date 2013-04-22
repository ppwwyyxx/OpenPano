// File: stitcher.cc
// Date: Mon Apr 22 21:15:50 2013 +0800
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
	for (int i = 0; i < newsize.y; i ++) filled[i] = new bool[newsize.x]();

	cout << newsize << endl;
	shared_ptr<Img> ret(new Img(newsize.x, newsize.y));
	ret->fill(Color::BLACK);

	for (int i = 0; i < img2->h; i ++)
		for (int j = 0; j < img2->w; j ++) {
			Color col = img2->get_pixel(i, j);
			Vec2D transformed = TransFormer::cal_project(trans, Coor(j, i));
			Coor newcoor = Coor(transformed.x, transformed.y) + offset;
			ret->set_pixel(newcoor.y, newcoor.x, col);
			filled[newcoor.y][newcoor.x] = true;
		}

	for (int i = 0; i < img1->h; i ++)
		for (int j = 0; j < img1->w; j ++) {
			Coor newcoor = Coor(j, i) + offset;
			Color col;
			if (filled[newcoor.y][newcoor.x])
				col = (ret->get_pixel(newcoor) + img1->get_pixel(i, j)) * 0.5;
			else
				col = img1->get_pixel(i, j);
			ret->set_pixel(newcoor.y, newcoor.x, col);
			filled[newcoor.y][newcoor.x] = true;
		}

	for (int i = 1; i < ret->h - 1; i ++)
		for (int j = 1; j < ret->w - 1; j ++)
			if (!filled[i][j]) {
				Coor black = Coor(j, i) - offset;
				Vec2D old = TransFormer::cal_project(inv, black);
				Vec2D check = TransFormer::cal_project(trans, Coor(old.x, old.y));
				/*
				 *cout << black << endl;
				 *cout << check << endl;
				 */
				if (between(old.x, 0, img2->w) && between(old.y, 0, img2->h))
					ret->set_pixel(i, j, img2->get_pixel(old.y, old.x));
			}

	free_2d<bool>(filled, newsize.y);
	return ret;
}
