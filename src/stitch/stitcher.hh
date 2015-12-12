// File: stitcher.hh
// Date: Sat May 04 22:36:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <utility>
#include <vector>
#include <type_traits>
#include "lib/mat.h"
#include "lib/utils.hh"
#include "lib/common.hh"
#include "lib/debugutils.hh"
#include "feature/feature.hh"
#include "stitcher_image.hh"
#include "camera.hh"

namespace pano {

// forward declaration
class Homography;
class MatchData;
struct MatchInfo;

class Stitcher {
	private:
		std::vector<Mat32f> imgs;
		// transformation and metadata of each image
		ConnectedImages bundle;

		// feature and keypoints of each image
		std::vector<std::vector<Descriptor>> feats;
		std::vector<std::vector<Vec2D>> keypoints;	// store coordinates in [-w/2,w/2]

		// 2d array of all matches
		// pairwise_matches[i][j].homo transform j to i
		std::vector<std::vector<MatchInfo>> pairwise_matches;

		// feature detector
		std::unique_ptr<FeatureDetector> feature_det;

		// get feature descriptor and keypoints for each image
		void calc_feature();

		// pairwise matching of all images
		void pairwise_match();
		// equivalent to pairwise_match when dealing with linear images
		void assume_linear_pairwise();

		// assign a center to be identity
		void assign_center();

		// build by estimating camera parameters
		void estimate_camera();

		// naively build panorama assuming linear imgs
		void build_linear_simple();

		// build panorama with cylindrical pre-warping
		void build_warp();

		// straighten camera parameters
		void straighten();

		// render the panorama
		Mat32f blend();

		// in cylindrical mode, search warping parameter for straightening
		float update_h_factor(float, float&, float&,
				std::vector<Homography>&,
				const std::vector<MatchData>&);
		// in cylindrical mode, perspective correction on the final image
		Mat32f perspective_correction(const Mat32f&);

		// helper template to prevent generic constructor being used as copy constructor
		template<typename A, typename B>
			using disable_if_same_or_derived =
			typename std::enable_if<
			!std::is_base_of<A, typename std::remove_reference<B>::type
			>::value
			>::type;

		// for debug
		void draw_matchinfo() const;
		void dump_matchinfo(const char*) const;
		void load_matchinfo(const char*);
	public:
		// universal reference constructor to initialize imgs
		template<typename U, typename X =
			disable_if_same_or_derived<Stitcher, U>>
			Stitcher(U&& i) : imgs(std::forward<U>(i)) {
				if (imgs.size() <= 1)
					error_exit(ssprintf("Cannot stitch with only %lu images.", imgs.size()));

				feature_det.reset(new SIFTDetector);

				// initialize bundle
				bundle.component.resize(imgs.size());
				REP(i, imgs.size())
					bundle.component[i].imgptr = &imgs[i];
			}

		Mat32f build();
};

}
