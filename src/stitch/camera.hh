//File: camera.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include "lib/matrix.hh"
#include "match_info.hh"

class Camera {
	public:
		Camera();

		Matrix K() const;


		double focal; // Focal length
		double aspect; // Aspect ratio
		double ppx; // Principal point X
		double ppy; // Principal point Y
		Matrix R; // Rotation
		Matrix t; // Translation

		static double estimate_focal(
				const std::vector<std::vector<MatchInfo>>& matches);
};
