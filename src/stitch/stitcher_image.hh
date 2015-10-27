//File: stitcher_image.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include "projection.hh"
#include "transform.hh"

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
