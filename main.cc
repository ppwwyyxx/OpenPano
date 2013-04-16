// File: main.cc
// Date: Tue Apr 16 10:29:52 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "image.hh"
#include "keypoint.hh"
#include "render/filerender.hh"
#include "planedrawer.hh"
#include "filter.hh"
using namespace std;

void show(shared_ptr<GreyImg> img) {
	Img res(*img);
	FileRender r(&res, "out.png");
	r.write(&res);
	r.finish();
}

int main(int argc, char* argv[]) {
	Img test("lenna.png");
	RenderBase* r = new FileRender(&test, "out.png");
	r->write(&test);
	cout << r->get_geo().w << r->get_geo().h << endl;
	PlaneDrawer pld(r);

	shared_ptr<Img> ptr(new Img(test));
	ScaleSpace ss(ptr, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.detect_extrema();
	ex.calc_dir();
	cout << ex.features.size() << endl;
	for (auto i : ex.features) {
		/*
		 *cout << i << endl;
		 */
		pld.cross(i.real_coor, 3);
	}
	r->finish();
	/*
	 *shared_ptr<GreyImg> ptr1 = Filter::GreyScale(ptr);
	 *shared_ptr<GreyImg> ptr2 = Filter::GaussianBlur(ptr1, 3);
	 */
}
