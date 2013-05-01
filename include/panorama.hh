// File: panorama.hh
// Date: Wed May 01 22:02:14 2013 +0800
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

		bool PANO = true;		// whether to generate panorama or planar

		imgptr get_trans();

	public:
		Panorama(const std::vector<imgptr>& i, bool m_PANO):
			imgs(i), PANO(m_PANO) {}

		imgptr get();

		static Matrix get_transform(const std::vector<Feature>&, const std::vector<Feature>&); // second -> first

		static std::vector<Feature> get_feature(imgptr &);

		static void cal_best_matrix(std::vector<imgptr>&, std::vector<Matrix>&, std::vector<std::pair<Vec2D, Vec2D>>&);
		static void cal_best_matrix_planar(std::vector<imgptr>&, std::vector<Matrix>&, std::vector<std::pair<Vec2D, Vec2D>>&);

		static void straighten_simple(std::vector<Matrix>& mat, const std::vector<imgptr>& imgs);

		static std::vector<std::pair<Vec2D, Vec2D>> cal_size(const std::vector<Matrix>& mat, const std::vector<imgptr>& imgs);

		static real_t update_h_factor(real_t, real_t&, real_t&,
				std::vector<Matrix>&,
				const std::vector<imgptr>&,
				const std::vector<std::vector<Feature>>&);
};
