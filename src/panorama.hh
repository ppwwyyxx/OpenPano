// File: panorama.hh
// Date: Sat May 04 22:36:30 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <utility>
#include <vector>
#include "lib/mat.h"
#include "matcher.hh"
#include "lib/matrix.hh"

class Panorama {
	private:
		std::vector<Mat32f> imgs;

		std::vector<Matrix> mat;

		std::vector<std::pair<Vec2D, Vec2D>> corners;

		bool CIRCLE = false;

	public:
		Panorama(const std::vector<Mat32f>& i):
			imgs(i){}

		Panorama(std::vector<Mat32f>&& i):
			imgs(i){}

		Mat32f get();

		static Matrix get_transform(
				const std::vector<Descriptor>&,
				const std::vector<Descriptor>&); // second -> first

		static std::vector<Descriptor> get_feature(const Mat32f&);

		void cal_best_matrix_pano();
		void cal_best_matrix();

		void straighten_simple();

		void cal_size();

		static float update_h_factor(float, float&, float&,
				std::vector<Matrix>&,
				const std::vector<Mat32f>&,
				const std::vector<std::vector<Descriptor>>&,
				const std::vector<MatchData>&);
};
