// File: feature.hh
// Date: Fri May 03 03:36:57 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once

#include "lib/config.hh"
#include "lib/geometry.hh"
#include <cstring>

struct Descriptor {
	Vec2D coor;
	std::vector<float> descriptor;
};
