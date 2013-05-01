// File: main.cc
// Date: Wed May 01 20:56:02 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "render/filerender.hh"
#include "planedrawer.hh"
#include "keypoint.hh"
#include "matcher.hh"
#include "gallery.hh"
#include "stitcher.hh"
#include "panorama.hh"
#include "transformer.hh"

#include <ctime>

using namespace std;
using namespace Magick;

bool TEMPDEBUG = false;

#define LABEL_LEN 7

inline real_t gen_rand()
{ return (real_t)rand() / RAND_MAX; }

vector<Feature> get_feature(imgptr& ptr)
{ return Panorama::get_feature(ptr); }

void test_feature(const char* fname) {
	shared_ptr<Img> test(new Img(fname));
	vector<Feature> ans = get_feature(test);
	RenderBase* r = new FileRender(test, "out.png");
	cout << r->get_geo().w << r->get_geo().h << endl;
	PlaneDrawer pld(r);

	cout << ans.size() << endl;
	for (auto i : ans)
		pld.arrow(toCoor(i.real_coor), i.dir, LABEL_LEN);
	r->finish();
	delete r;
}


void gallery(const char* f1, const char* f2) {
	list<Image> imagelist;
	Image pic1(f1);
	Image pic2(f2);
	imagelist.push_back(pic1);
	imagelist.push_back(pic2);
	Gallery ga(imagelist);

	shared_ptr<Img> test(new Img(ga.get()));
	RenderBase* r = new FileRender(test, "out.png");
	PlaneDrawer pld(r);

	vector<Feature> ans = get_feature(test);
	for (auto i : ans) pld.arrow(toCoor(i.real_coor), i.dir, LABEL_LEN);

	shared_ptr<Img> ptr1(new Img(pic1));
	shared_ptr<Img> ptr2(new Img(pic2));
	vector<Feature> feat1 = get_feature(ptr1);
	vector<Feature> feat2 = get_feature(ptr2);

	Matcher match(feat1, feat2);
	auto ret = match.match();
	for (auto &x : ret.data) {
		pld.set_color(::Color(gen_rand(), gen_rand(), gen_rand()));
		pld.circle(toCoor(x.first), LABEL_LEN);
		pld.circle(toCoor(x.second) + Coor(ptr1->w, 0), LABEL_LEN);
		pld.line(toCoor(x.first), toCoor(x.second) + Coor(ptr1->w, 0));
	}

	r->finish();
	delete r;
}

void test_memory(const char* fname) {
	shared_ptr<Img> test(new Img(fname));
	get_feature(test);
	return;
}

void test_transform(const char* f1, const char* f2) {
	imgptr ptr1(new Img(f1));
	imgptr ptr2(new Img(f2));

	Panorama p({ptr1, ptr2});
	shared_ptr<Img> res = p.get();

	RenderBase* r = new FileRender(res, "out.png");
	r->finish();
	delete r;
}

void final(int argc, char* argv[]) {
	vector<imgptr> imgs;
	REPL(i, 1, argc) {
		imgptr ptr(new Img(argv[i]));
		imgs.push_back(ptr);
	}
	Panorama p(imgs);
	imgptr res = p.get();

	res->crop();
	RenderBase* r = new FileRender(res, "out.png");
	r->finish();
	delete r;
}



int main(int argc, char* argv[]) {
	srand(time(NULL));
	/*
	 *test_feature(argv[1]);
	 */
	/*
	 *test_transform(argv[1], argv[2]);
	 */
	final(argc, argv);
}
