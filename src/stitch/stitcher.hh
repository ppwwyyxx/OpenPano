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

// forward declaration
class Homography;
namespace feature { class MatchData; }

namespace stitch {

struct MatchInfo;

class Stitcher {
	private:
		std::vector<Mat32f> imgs;
		// transformation and metadata of each image
		ConnectedImages bundle;
		// feature of each image
		std::vector<std::vector<feature::Descriptor>> feats;
		// 2d array of all matches
		// pairwise_matches[i][j].homo transform j to i
		std::vector<std::vector<MatchInfo>> pairwise_matches;
		// camera intrinsic params for all images
		std::vector<Camera> cameras;
		// feature detector
		std::unique_ptr<feature::FeatureDetector> feature_det;

		// get feature descriptor for each image
		void calc_feature();

		// pairwise matching of all images
		void pairwise_match();
		// equivalent to pairwise_match when dealing with linear images
		void assume_linear_pairwise();

		// assign a center to be identity
		void assign_center();

		// build bundle by estimating camera parameters
		void estimate_camera();

		// naively build bundle assuming linear imgs
		void build_bundle_linear_simple();

		// build bundle with cylindrical pre-warping
		void build_bundle_warp();

		// find MST from pair-matches, by confidence
		bool max_spanning_tree(std::vector<std::vector<int>>& graph);

		// straighten camera parameters
		void straighten();

		// blend the bundle
		Mat32f blend();

		// in cylindrical mode, search warping parameter for straightening
		float update_h_factor(float, float&, float&,
				std::vector<Homography>&,
				const std::vector<feature::MatchData>&);
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
		void draw_matchinfo();
		void dump_matchinfo(const char*);
		void load_matchinfo(const char*);
	public:
		// universal reference constructor to initialize imgs
		template<typename U, typename X =
			disable_if_same_or_derived<Stitcher, U>>
			Stitcher(U&& i) : imgs(std::forward<U>(i)) {
				if (imgs.size() <= 1)
					error_exit(ssprintf("Cannot stitch with only %lu images.", imgs.size()));

				feature_det.reset(new feature::SIFTDetector);

				// initialize members
				pairwise_matches.resize(imgs.size());
				for (auto& k : pairwise_matches) k.resize(imgs.size());
				feats.resize(imgs.size());
				cameras.resize(imgs.size());
				bundle.component.resize(imgs.size());
				REP(i, imgs.size())
					bundle.component[i].imgptr = &imgs[i];
			}

		Mat32f build();
};

}
