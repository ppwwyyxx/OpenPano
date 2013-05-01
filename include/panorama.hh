// File: panorama.hh
// Date: Wed May 01 12:26:16 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <memory>
#include <utility>
#include "image.hh"
#include "feature.hh"
#include "matrix.hh"

class Panorama {
	private:
		std::vector<imgptr> imgs;

		imgptr get_trans();


	public:
		Panorama(const std::vector<imgptr>& i):
			imgs(i) {}

		imgptr get();

		static Matrix get_transform(const std::vector<Feature>&, const std::vector<Feature>&); // second -> first

		static std::vector<Feature> get_feature(imgptr &);

		void cal_best_matrix(std::vector<imgptr>&, std::vector<Matrix>&, Vec2D&, Vec2D&) const;

		static void straighten_simple(std::vector<Matrix>& mat, const std::vector<imgptr>& imgs);

		static std::pair<Vec2D, Vec2D> cal_size(const std::vector<Matrix>& mat, const std::vector<imgptr>& imgs);
};
