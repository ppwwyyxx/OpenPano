// File: filter.hh
// Date: Fri May 03 04:47:02 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include "image.hh"

class Filter {
	public:
		static std::shared_ptr<GreyImg> GaussianBlur(const std::shared_ptr<GreyImg>, real_t);
		static std::shared_ptr<GreyImg> GreyScale(const std::shared_ptr<Img>);
		static real_t to_grey(const ::Color&);
};
