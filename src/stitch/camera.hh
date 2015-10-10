//File: camera.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include "lib/matrix.hh"
#include "match_info.hh"

class Camera {
	public:
		Camera();

		Homography K() const;

		double focal = 1; // Focal length
		double aspect = 1; // Aspect ratio
		double ppx = 0; // Principal point X
		double ppy = 0; // Principal point Y
		Homography R; // Rotation
		//Vec t; // Translation

		static double estimate_focal(
				const std::vector<std::vector<MatchInfo>>& matches);

		static void rotation_to_angle(const Homography& r, double& rx, double& ry, double& rz);

		static void angle_to_rotation(double rx, double ry, double rz, Homography& r);
};
