// File: main.cc
// Date: Tue Apr 23 11:37:49 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "render/filerender.hh"
#include "planedrawer.hh"
#include "keypoint.hh"
#include "matcher.hh"
#include "gallery.hh"
#include "stitcher.hh"
#include "transformer.hh"
#include <ctime>

using namespace std;
using namespace Magick;

#define LABEL_LEN 7

inline real_t gen_rand()
{ return (real_t)rand() / RAND_MAX; }

vector<Feature> get_feature(shared_ptr<Img> ptr) {
	ScaleSpace ss(ptr, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.work();
	return move(ex.features);
}

void test_feature(const char* fname) {
	shared_ptr<Img> test(new Img(fname));
	RenderBase* r = new FileRender(test, "out.png");
	cout << r->get_geo().w << r->get_geo().h << endl;
	PlaneDrawer pld(r);

	vector<Feature> ans = get_feature(test);
	cout << ans.size() << endl;
	for (auto i : ans)
		pld.arrow(toCoor(i.real_coor), i.dir, LABEL_LEN);
	r->finish();
	/*
	 *shared_ptr<GreyImg> ptr1 = Filter::GreyScale(ptr);
	 *shared_ptr<GreyImg> ptr2 = Filter::GaussianBlur(ptr1, 3);
	 */
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
	Image pic1(f1);
	Image pic2(f2);

	shared_ptr<Img> ptr1(new Img(pic1));
	shared_ptr<Img> ptr2(new Img(pic2));
	vector<Feature> feat1 = get_feature(ptr1);
	vector<Feature> feat2 = get_feature(ptr2);

	HWTimer timer;
	Matcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("match time: %lf secs\n", timer.get_sec());

	TransFormer transf(ret);
	Matrix trans = transf.get_transform();
	print_debug("transf time: %lf secs\n", timer.get_sec());

	/*
	 *real_t x = M_PI / 6;
	 *Matrix rot(3, 3);
	 *rot.get(0, 0) = rot.get(1, 1) = cos(x), rot.get(0, 1) = sin(x);
	 *rot.get(1, 0) = -sin(x);
	 *rot.get(2, 2) = 1;
	 */
	Stitcher st(ptr1, ptr2, trans);
	shared_ptr<Img> res = st.stitch();
	print_debug("stitch time: %lf secs\n", timer.get_sec());

	RenderBase* r = new FileRender(res, "out.png");
	r->finish();
	delete r;
}


int main(int argc, char* argv[]) {
	srand(time(NULL));
	test_feature(argv[1]);
	/*
	 *gallery(argv[1], argv[2]);
	 */
	/*
	 *test_transform(argv[1], argv[2]);
	 */
}
