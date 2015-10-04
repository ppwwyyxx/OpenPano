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
class Descriptor;

// An image to be stitched
struct ImageComponent {
	Homography homo,	// from me to identity
						 homo_inv;	// from identity to me
	//int idx;	// its original index
	ImageComponent(){}
	ImageComponent(const Homography& h): homo(h) {}
};

struct ConnectedImages {
	enum ProjectionMethod {
		flat,
		cylindrical
	};
	ProjectionMethod proj_method;
	int identity_idx;

	Vec2D proj_min, proj_max;
	std::vector<ImageComponent> component;

	void update_proj_range();

	void calc_inverse_homo();
};

class Stitcher {
	private:
		std::vector<Mat32f> imgs;

		ConnectedImages bundle;

		// feature for each image
		std::vector<std::vector<Descriptor>> feats;

		std::vector<std::pair<Vec2D, Vec2D>> corners;

		bool circle_detected = false;

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
		void cal_best_matrix_pano();
		void calc_matrix_simple();

		//void straighten_simple();

		void cal_size();

		static float update_h_factor(float, float&, float&,
				std::vector<Homography>&,
				const std::vector<Mat32f>&,
				const std::vector<std::vector<Descriptor>>&,
				const std::vector<MatchData>&);
};
