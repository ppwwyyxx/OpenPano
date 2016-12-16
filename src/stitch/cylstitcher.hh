//File: cylstitcher.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once

#include "stitcherbase.hh"
#include "stitcher_image.hh"
#include "common/common.hh"
namespace pano {

// forward declaration
class Homography;
class MatchData;
struct MatchInfo;

// a stitcher which warps images to cyilnder before stitching
class CylinderStitcher : public StitcherBase {
	protected:
		ConnectedImages bundle;

		// build panorama with cylindrical pre-warping
		void build_warp();

		// in cylindrical mode, search warping parameter for straightening
		float update_h_factor(float, float&, float&,
				std::vector<Homography>&,
				const std::vector<MatchData>&);
		// in cylindrical mode, perspective correction on the final image
		Mat32f perspective_correction(const Mat32f&);

	public:
		template<typename U, typename X =
			disable_if_same_or_derived<CylinderStitcher, U>>
			CylinderStitcher(U&& i) : StitcherBase(std::forward<U>(i)) {
				bundle.component.resize(imgs.size());
				REP(i, imgs.size())
					bundle.component[i].imgptr = &imgs[i];
			}

		virtual Mat32f build();
};


}
