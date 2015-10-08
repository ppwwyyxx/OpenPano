// File: stitcher.hh
// Date: Sat May 04 22:36:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <utility>
#include <vector>
#include "lib/mat.h"
#include "lib/matrix.hh"
#include "lib/utils.hh"
#include "lib/geometry.hh"
#include "transform.hh"
#include "match_info.hh"
#include "feature/feature.hh"
#include "stitcher_image.hh"

// forward declaration
namespace feature { class MatchData; }

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
		// connection between images. stored as adj table
		std::vector<std::vector<int>> graph;

		// feature detector
		std::unique_ptr<feature::FeatureDetector> feature_det;

		template<typename A, typename B>
			using disable_if_same_or_derived =
			typename std::enable_if<
			!std::is_base_of<A, typename std::remove_reference<B>::type
			>::value
			>::type;

		// get feature descriptor for each image
		void calc_feature();

		// pairwise matching of all images
		void pairwise_match();
		// equivalent to pairwise_match when dealing with panorama
		void assume_pano_pairwise();

		// build bundle assuming linear imgs
		void build_bundle_linear_simple();
		// build bundle with pre-warping
		void build_bundle_warp();


		Mat32f blend();

		// old hacking code
		void calc_matrix_pano();
		void straighten_simple();
		float update_h_factor(float, float&, float&,
				std::vector<Homography>&,
				const std::vector<feature::MatchData>&);

		Homography get_transform(int f1, int f2) const; // second -> first

	public:
		// universal reference constructor to initialize imgs
		// and avoid using this template as copy-constructor
		template<typename U, typename X =
			disable_if_same_or_derived<Stitcher, U>>
			Stitcher(U&& i) : imgs(std::forward<U>(i)) {
				if (imgs.size() <= 1)
					error_exit(ssprintf("Cannot stitch with only %lu images.", imgs.size()));

				if (USE_SIFT)
					feature_det.reset(new feature::SIFTDetector);
				else
					feature_det.reset(new feature::BRIEFDetector);

				graph.resize(imgs.size());
				pairwise_matches.resize(imgs.size());
				for (auto& k : pairwise_matches) k.resize(imgs.size());

				feats.resize(imgs.size());
			}

		Mat32f build();
		static std::vector<feature::Descriptor> get_feature(const Mat32f&);
};
