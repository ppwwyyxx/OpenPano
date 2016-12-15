//File: stitcherbase.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include <memory>
#include "lib/mat.h"
#include "lib/geometry.hh"
#include "feature/feature.hh"
#include "imageref.hh"
#include "common/common.hh"

namespace pano {

// forward declaration
class Homography;
class MatchData;
struct MatchInfo;

class StitcherBase {
	protected:
		// helper template to prevent generic constructor being used as copy constructor
		template<typename A, typename B>
			using disable_if_same_or_derived =
			typename std::enable_if<
			!std::is_base_of<A, typename std::remove_reference<B>::type
			>::value
			>::type;

		std::vector<ImageRef> imgs;

		// feature and keypoints of each image
    // feats[k][i].coor == keypoints[k][i]
    // keypoints only work as a storage of feats, because feats will be freed
		std::vector<std::vector<Descriptor>> feats;	// [-w/2,w/2]
		std::vector<std::vector<Vec2D>> keypoints;	// store coordinates in [-w/2,w/2]

		// feature detector
		std::unique_ptr<FeatureDetector> feature_det;

		// get feature descriptor and keypoints for each image
		void calc_feature();

		void free_feature();

	public:
		// universal reference constructor to initialize imgs
		template<typename U, typename X =
			disable_if_same_or_derived<StitcherBase, U>>
			StitcherBase(U&& i) {
				/*
				 *if (imgs.size() <= 1)
				 *  error_exit(ssprintf("Cannot stitch with only %lu images.", imgs.size()));
				 */
				for (auto& n : i)
					imgs.emplace_back(n);

				feature_det.reset(new SIFTDetector);
			}

		StitcherBase(const StitcherBase&) = delete;
		StitcherBase& operator = (const StitcherBase&) = delete;

		virtual Mat32f build() = 0;

		virtual ~StitcherBase() = default;
};

}
