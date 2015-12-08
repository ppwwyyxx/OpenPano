//File: camera_estimator.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "match_info.hh"


namespace stitch {
class Camera;

class CameraEstimator {
	public:

		CameraEstimator(
				const std::vector<std::vector<MatchInfo>>& matches,
				const std::vector<Shape2D>& image_shapes) :
				n(matches.size()),
				matches(matches),
				shapes(image_shapes),
				cameras(matches.size())
			{ m_assert(matches.size() == shapes.size()); }

		std::vector<Camera> estimate();

	protected:
		typedef std::vector<std::vector<int>> Graph;

		int n;	// nr_img
		const std::vector<std::vector<MatchInfo>>& matches;
		const std::vector<Shape2D>& shapes;

		std::vector<Camera> cameras;

		Graph max_spanning_tree();

		void propagate_rotation(const Graph& graph);
};


}
