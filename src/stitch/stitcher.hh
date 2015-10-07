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
#include "feature/feature.hh"

namespace feature { class MatchData; }

// forward declaration

struct ConnectedImages {
	struct Range {
		Vec2D min, max;
		Range(){}
		Range(const Vec2D& a, const Vec2D& b): min(a), max(b) {}
	};

	// -- projections
	enum ProjectionMethod { flat, cylindrical, spherical };
	ProjectionMethod proj_method;

	Range proj_range;	// in identity image coordinate

	// update range of projection of all transformations
	void update_proj_range();

	projector::homo2proj_t get_homo2proj() const;
	projector::proj2homo_t get_proj2homo() const;

	int identity_idx;

	// -- image transformations and metadata

	struct ImageComponent {
		Homography homo,			// from me to identity
							 homo_inv;	// from identity to me

		// point to the original image
		Mat32f* imgptr;

		// range after projected to identity frame
		Range range;

		ImageComponent(){}
		ImageComponent(Mat32f* img):imgptr(img) {}
	};

	std::vector<ImageComponent> component;

	// inverse all homographies
	void calc_inverse_homo();

};

class Stitcher {
	private:
		std::vector<Mat32f> imgs;
		// transformation and metadata of each image
		ConnectedImages bundle;
		// feature of each image
		std::vector<std::vector<feature::Descriptor>> feats;

		std::unique_ptr<feature::FeatureDetector> feature_det;

		template<typename A, typename B>
			using disable_if_same_or_derived =
			typename std::enable_if<
			!std::is_base_of<A, typename std::remove_reference<B>::type
			>::value
			>::type;

		void calc_matrix_pano();
		void calc_matrix_simple();

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
			}

		Mat32f build();

		Homography get_transform(int f1, int f2) const; // second -> first

		static std::vector<feature::Descriptor> get_feature(const Mat32f&);

		void calc_feature();

		// calculate feature and pairwise transform
		void calc_transform();

		void straighten_simple();

		Mat32f blend();

		float update_h_factor(float, float&, float&,
				std::vector<Homography>&,
				const std::vector<feature::MatchData>&);
};
