// File: panorama.hh
// Date: Sun Apr 28 19:16:23 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <memory>
#include "image.hh"
#include "feature.hh"
#include "matrix.hh"

class Panorama {
	private:
		std::vector<imgptr> imgs;

		imgptr get_trans() const;

	public:
		Panorama(const std::vector<imgptr>& i) { imgs = i; }

		imgptr get() const;

		static Matrix get_transform(const std::vector<Feature>&, const std::vector<Feature>&); // second -> first

		static std::vector<Feature> get_feature(imgptr);
};
