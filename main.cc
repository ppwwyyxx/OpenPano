// File: main.cc
// Date: Sat Apr 20 10:38:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "keypoint.hh"
#include "render/filerender.hh"
#include "planedrawer.hh"
#include "filter.hh"
#include "gallery.hh"
#include <ctime>

using namespace std;
using namespace Magick;

#define LABEL_LEN 7

inline real_t gen_rand()
{ return (real_t)rand() / RAND_MAX; }

void show(shared_ptr<GreyImg> img) {
	Img res(*img);
	FileRender r(&res, "out.png");
	r.write(&res);
	r.finish();
}

vector<Feature> get_feature(shared_ptr<Img> ptr) {
	ScaleSpace ss(ptr, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.work();
	return move(ex.features);
}

void test_feature(const char* fname) {
	Img test(fname);
	RenderBase* r = new FileRender(&test, "out.png");
	r->write(&test);
	cout << r->get_geo().w << r->get_geo().h << endl;
	PlaneDrawer pld(r);

	shared_ptr<Img> ptr(new Img(test));
	vector<Feature> ans = get_feature(ptr);
	cout << ans.size() << endl;
	for (auto i : ans) {
		pld.arrow(i.real_coor, i.dir, LABEL_LEN);
		for (real_t x : i.descriptor)
			cout << x << " ";
		cout << endl;
	}
	r->finish();
	/*
	 *shared_ptr<GreyImg> ptr1 = Filter::GreyScale(ptr);
	 *shared_ptr<GreyImg> ptr2 = Filter::GaussianBlur(ptr1, 3);
	 */
}

int cal_dist(Feature& x, Feature& y) {
	int ans = 0;
	for (int i = 0; i < DESC_LEN; i ++)
		ans += abs(x.descriptor[i] - y.descriptor[i]);
	return ans;
}

void gallery(const char* f1, const char* f2) {
	list<Image> imagelist;
	Image pic1(f1);
	Image pic2(f2);
	imagelist.push_back(pic1);
	imagelist.push_back(pic2);
	Gallery ga(imagelist);

	Img test(ga.get());
	RenderBase* r = new FileRender(&test, "out.png");
	PlaneDrawer pld(r);
	r->write(&test);

	shared_ptr<Img> ptr(new Img(test));
	vector<Feature> ans = get_feature(ptr);
	for (auto i : ans) pld.arrow(i.real_coor, i.dir, LABEL_LEN);

	shared_ptr<Img> ptr1(new Img(pic1));
	shared_ptr<Img> ptr2(new Img(pic2));
	vector<Feature> feat1 = get_feature(ptr1);
	vector<Feature> feat2 = get_feature(ptr2);

	int cnt = 0, len = feat1.size();
#pragma omp parallel for schedule(dynamic)
	for (int k = 0; k < len; k ++) {
		Feature& i = feat1[k];
		int min = numeric_limits<int>::max();
		Coor mincoor;
		for (auto &j : feat2) {
			int dist = cal_dist(i, j);
			if (dist < min) {
				min = dist;
				mincoor = j.real_coor;
			}
		}
		if (min > 400) continue;
		mincoor.x += ptr1->w;

#pragma omp critical
		{
			pld.set_color(::Color(gen_rand(), gen_rand(), gen_rand()));
			pld.circle(i.real_coor, LABEL_LEN);
			pld.circle(mincoor, LABEL_LEN);
			pld.line(i.real_coor, mincoor);
			cnt ++;
		}
	}
	cout << "match: " << cnt << endl;

	r->finish();
}

int main(int argc, char* argv[]) {
	srand(time(NULL));
	/*
	 *test_feature(argv[1]);
	 */
	gallery(argv[1], argv[2]);
}
