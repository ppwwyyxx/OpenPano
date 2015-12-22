//File: camera_estimator.hh
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <functional>

namespace pano {

struct MatchInfo;
struct Shape2D;
class Camera;

class CameraEstimator {
	public:
		CameraEstimator(
				const std::vector<std::vector<MatchInfo>>& matches,
				const std::vector<Shape2D>& image_shapes);

		~CameraEstimator();

		CameraEstimator(const CameraEstimator&) = delete;
		CameraEstimator& operator = (const CameraEstimator&) = delete;

		std::vector<Camera> estimate();

	private:
		typedef std::vector<std::vector<int>> Graph;

		int n;	// nr_img
		const std::vector<std::vector<MatchInfo>>& matches;
		const std::vector<Shape2D>& shapes;

		std::vector<Camera> cameras;

		void traverse(
				std::function<void(int)> callback_init_node,
				std::function<void(int, int)> callback_add_edge);
};


}
