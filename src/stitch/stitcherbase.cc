//File: stitcherbase.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "stitcherbase.hh"
#include "lib/timer.hh"

namespace pano {

void StitcherBase::calc_feature() {
	GuardedTimer tm("calc_feature()");
	feats.resize(imgs.size());
	keypoints.resize(imgs.size());
	// detect feature
#pragma omp parallel for schedule(dynamic)
	REP(k, imgs.size()) {
		feats[k] = feature_det->detect_feature(imgs[k]);
		if (feats[k].size() == 0)	// TODO delete the image
			error_exit(ssprintf("Cannot find feature in image %lu!\n", k));
		print_debug("Image %lu has %lu features\n", k, feats[k].size());
		keypoints[k].resize(feats[k].size());
		REP(i, feats[k].size())
			keypoints[k][i] = feats[k][i].coor;
	}
}

}
