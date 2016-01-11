// File: color.cc
// Date: Sat May 04 12:50:29 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>


#include "color.hh"
using namespace std;

namespace pano {

const Color Color::BLACK(0, 0, 0),
			Color::WHITE(1,1,1),
			Color::RED(1, 0, 0),
			Color::BLUE(0, 0, 1),
			Color::NO(-1, -1, -1);

}
