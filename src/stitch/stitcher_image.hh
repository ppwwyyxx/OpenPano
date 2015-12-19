//File: stitcher_image.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include "lib/mat.h"
#include "projection.hh"
#include "homography.hh"

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
	};

	// -- projections
	enum ProjectionMethod { flat, cylindrical, spherical };
	ProjectionMethod proj_method;

	Range proj_range;	// in identity image coordinate

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

	// update range of projection of all transformations
	void update_proj_range();

	homo2proj_t get_homo2proj() const;
	proj2homo_t get_proj2homo() const;


	// apply translation to homography
	// originally all homo operates on half-shifted coordinate
	// after calling this function they operate on image coordinate
	void shift_all_homo();

	// inverse all homographies
	void calc_inverse_homo();

};

}
