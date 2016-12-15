//File: camera_estimator.hh
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <functional>
#include "common/common.hh"
namespace pano {

struct MatchInfo;
struct Shape2D;
class Camera;

class CameraEstimator {
	public:
		CameraEstimator(
				std::vector<std::vector<MatchInfo>>& matches,
				const std::vector<Shape2D>& image_shapes);

		~CameraEstimator();

		CameraEstimator(const CameraEstimator&) = delete;
		CameraEstimator& operator = (const CameraEstimator&) = delete;

		std::vector<Camera> estimate();

    void estimate_focal();

	private:
		typedef std::vector<std::vector<int>> Graph;

		int n;	// nr_img
		// matches will be modified to filter-out low-quality matches
		std::vector<std::vector<MatchInfo>>& matches;
		const std::vector<Shape2D>& shapes;

    // hold a copy of all cameras
		std::vector<Camera> cameras;

		void traverse(
				std::function<void(int)> callback_init_node,
				std::function<void(int, int)> callback_add_edge);
};


}
