// File: panorama.hh
// Date: Sat May 04 22:32:37 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <utility>
#include <vector>
#include "image.hh"
#include "matcher.hh"
#include "matrix.hh"

class Panorama {
	private:
		std::vector<imgptr> imgs;

		std::vector<Matrix> mat;

		std::vector<std::pair<Vec2D, Vec2D>> corners;
		imgptr get_trans();

		bool CIRCLE = false;

	public:
		Panorama(std::vector<imgptr>& i):
			imgs(i){}

		imgptr get();

		static Matrix get_transform(const std::vector<Feature>&, const std::vector<Feature>&); // second -> first

		static std::vector<Feature> get_feature(imgptr &);

		void cal_best_matrix_pano();
		void cal_best_matrix();

		void straighten_simple();

		void cal_size();

		static real_t update_h_factor(real_t, real_t&, real_t&,
				std::vector<Matrix>&,
				const std::vector<imgptr>&,
				const std::vector<std::vector<Feature>>&,
				const std::vector<MatchData>&);
};
