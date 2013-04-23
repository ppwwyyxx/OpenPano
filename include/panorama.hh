// File: panorama.hh
// Date: Tue Apr 23 17:55:16 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include <memory>
#include "image.hh"
#include "feature.hh"
#include "matrix.hh"

class Panorama {
	private:
		std::vector<imgptr> imgs;

	public:
		Panorama(const std::vector<std::shared_ptr<Img>>& i) { imgs = i; }

		std::shared_ptr<Img> get() const;

		static Matrix get_transform(const std::vector<Feature>&, const std::vector<Feature>&); // second -> first

		static std::vector<Feature> get_feature(imgptr);
};
