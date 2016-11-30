//File: camera.hh
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#pragma once
#include <vector>
#include "homography.hh"
#include "common/common.hh"
namespace pano {
struct MatchInfo;

// TODO might not need aspect any more
class Camera {
	public:
		Camera();
		Camera(const Camera&) = default;
		Camera& operator = (const Camera&) = default;

		// return the intrinsic matrix
		Homography K() const;

		Homography Kinv() const { return K().inverse(); }

		Homography Rinv() const { return R.transpose(); }

		double focal = 1; // Focal length
		double aspect = 1; // Aspect ratio
		double ppx = 0; // Principal point X
		double ppy = 0; // Principal point Y
		Homography R; // Rotation

		static double estimate_focal(
				const std::vector<std::vector<MatchInfo>>& matches);

		static void rotation_to_angle(const Homography& r, double& rx, double& ry, double& rz);

		static void angle_to_rotation(double rx, double ry, double rz, Homography& r);

		static void straighten(std::vector<Camera>&);

		friend std::ostream& operator << (std::ostream& os, const Camera& c) {
			os << "focal=" << c.focal << ", ppx=" << c.ppx << ", ppy="
				<< c.ppy << ", R=" << c.R;
			double rx, ry, rz;
			Camera::rotation_to_angle(c.R, rx, ry, rz);
			os << ", theta=" << rx << " " << ry << " " << rz;
			return os;
		}
};
}
