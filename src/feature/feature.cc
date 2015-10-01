//File: feature.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "feature.hh"
#include "extrema.hh"
#include "dog.hh"
#include "keypoint.hh"
using namespace std;

vector<Descriptor> detect_SIFT(const Mat32f& mat) {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);
	KeyPoint ex(sp, ss);
	ex.work();
	return ex.get_sift_descriptor();
}


vector<SSPoint> detect_extrema(const Mat32f& mat) {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace dog(ss);
	return ExtremaDetector(dog).get_extrema();
}
