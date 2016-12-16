//File: stitcher_image.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include <cassert>
#include "lib/mat.h"
#include "projection.hh"
#include "homography.hh"
#include "imageref.hh"
#include "common/common.hh"

namespace pano {

/// A group of connected images, and metadata for stitching
struct ConnectedImages {
	ConnectedImages() = default;
	ConnectedImages(const ConnectedImages&) = delete;
	ConnectedImages& operator = (const ConnectedImages&) = delete;

	struct Range {
		Vec2D min, max;
		Range(){}
		Range(const Vec2D& a, const Vec2D& b): min(a), max(b) {}
		Vec2D size() const { return max - min; }
	};

	// -- projections
	enum ProjectionMethod { flat, cylindrical, spherical };
	ProjectionMethod proj_method;

	Range proj_range;	// in identity image coordinate

	int identity_idx;	// WARNING, this should be the index within this bundle

	// -- image transformations and metadata

	struct ImageComponent {
		Homography homo,			// from me (point in image plane, 2d) to point in space (P,3D)
							 homo_inv;	// transform from point in space to me (point in plane):
                          // p = K * R * P

		// point to the original image
		ImageRef* imgptr;

		// range after projected to identity frame
		Range range;

		ImageComponent() {}
		ImageComponent(ImageRef* img):imgptr(img) {}
	};

	std::vector<ImageComponent> component;

	// update range of projection of all transformations
	void update_proj_range();

	homo2proj_t get_homo2proj() const {
		switch (proj_method) {
			case ProjectionMethod::flat:
				return flat::homo2proj;
			case ProjectionMethod::cylindrical:
				return cylindrical::homo2proj;
			case ProjectionMethod::spherical:
				return spherical::homo2proj;
		}
		assert(false);
	}
	proj2homo_t get_proj2homo() const {
		switch (proj_method) {
			case ProjectionMethod::flat:
				return flat::proj2homo;
			case ProjectionMethod::cylindrical:
				return cylindrical::proj2homo;
			case ProjectionMethod::spherical:
				return spherical::proj2homo;
		}
		assert(false);
	}


	// apply translation to homography
	// originally all homo operates on half-shifted coordinate
	// after calling this function they operate on image coordinate
	void shift_all_homo();

	// inverse all homographies
	void calc_inverse_homo();

	Mat32f blend() const;

	Vec2D get_final_resolution() const;
};

}
