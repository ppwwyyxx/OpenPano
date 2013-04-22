// File: stitcher.cc
// Date: Mon Apr 22 19:12:14 2013 +0800
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
	shared_ptr<Img> ret(new Img(newsize.x, newsize.y));
	/*
	 *for (int i = 0; i < h; i ++)
	 *    for (int j = 0; j < h; j ++)
	 */
}
