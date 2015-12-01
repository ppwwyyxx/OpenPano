//File: feature.cc
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "feature.hh"

#include "extrema.hh"
#include "orientation.hh"
#include "sift.hh"
#include "dog.hh"
#include "brief.hh"
#include "lib/imgproc.hh"
using namespace std;
using namespace config;


namespace feature {

vector<Descriptor> FeatureDetector::detect_feature(const Mat32f& img) const {
	auto ret = do_detect_feature(img);
	// convert scale-coordinate to half-offset image coordinate
	for (auto& d: ret) {
		d.coor.x = (d.coor.x - 0.5) * img.width();
		d.coor.y = (d.coor.y - 0.5) * img.height();
	}
	return ret;
}

vector<Descriptor> SIFTDetector::do_detect_feature(const Mat32f& mat) const {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	OrientationAssign ort(sp, ss, keyp);
	keyp = ort.work();
	SIFT sift(ss, keyp);
	auto ret = sift.get_descriptor();
	return ret;
}

BRIEFDetector::BRIEFDetector() {
	pattern.reset(new BriefPattern(
				BRIEF::gen_brief_pattern(BRIEF_PATH_SIZE, BRIEF_NR_PAIR)));
}

BRIEFDetector::~BRIEFDetector() {}

vector<Descriptor> BRIEFDetector::do_detect_feature(const Mat32f& mat) const {
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace sp(ss);

	ExtremaDetector ex(sp);
	auto keyp = ex.get_extrema();
	//OrientationAssign ort(sp, ss, keyp);
	//keyp = ort.work();
	BRIEF brief(mat, keyp, *pattern);

	auto ret = brief.get_descriptor();
	return ret;
}

}
