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

// forward declaration
class MatchData;
struct Descriptor;

struct ConnectedImages {
	enum ProjectionMethod {
		flat,
		cylindrical,
		spherical
	};
	ProjectionMethod proj_method;

	int identity_idx;

	struct Range {
		Vec2D min, max;
		Range(const Vec2D& a, const Vec2D& b): min(a), max(b) {}
		Range(){}
	};

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
	Vec2D proj_min, proj_max;	// in identity image coordinate

	// update range of projection of all transformations
	void update_proj_range();

	// inverse all homographies
	void calc_inverse_homo();

	projector::homo2proj_t get_homo2proj() const;
	projector::proj2homo_t get_proj2homo() const;
};

class Stitcher {
	private:
		std::vector<Mat32f> imgs;

		ConnectedImages bundle;

		// feature for each image
		std::vector<std::vector<Descriptor>> feats;

		template<typename A, typename B>
			using disable_if_same_or_derived =
			typename std::enable_if<
			!std::is_base_of<A, typename std::remove_reference<B>::type
			>::value
			>::type;

	public:
		// universal reference constructor to initialize imgs
		// and avoid using this template as copy-constructor
		template<typename U, typename X =
			disable_if_same_or_derived<Stitcher, U>>
			Stitcher(U&& i) : imgs(std::forward<U>(i)) {
				if (imgs.size() <= 1)
					error_exit(ssprintf("Cannot stitch with only %lu images.", imgs.size()));
			}

		Mat32f build();

		Homography get_transform(int f1, int f2) const; // second -> first

		static std::vector<Descriptor> get_feature(const Mat32f&);

		void calc_feature();

		// calculate feature and pairwise transform
		void calc_transform();
		void calc_matrix_pano();
		void calc_matrix_simple();

		void straighten_simple();

		Mat32f blend();

		float update_h_factor(float, float&, float&,
				std::vector<Homography>&,
				const std::vector<MatchData>&);
};
