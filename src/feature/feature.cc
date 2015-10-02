//File: feature.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "feature.hh"

#include "extrema.hh"
#include "orientation.hh"
#include "sift.hh"
#include "dog.hh"
//#include "keypoint.hh"
using namespace std;

vector<Descriptor> detect_SIFT(const Mat32f& mat) {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

#if 0
	KeyPoint ex(sp, ss);
	ex.work();
	auto ret = ex.get_sift_descriptor();
#else
	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	OrientationAssign ort(sp, ss, keyp);
	keyp = ort.work();
	SIFT sift(ss, keyp);
	auto ret = sift.get_descriptor();
#endif
	cout << "number of features: " << ret.size() << endl;
	return ret;
}


vector<SSPoint> detect_extrema(const Mat32f& mat) {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace dog(ss);
	return ExtremaDetector(dog).get_extrema();
}
