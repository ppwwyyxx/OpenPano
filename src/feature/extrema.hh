//File: extrema.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "dog.hh"
#include "lib/mat.h"
#include "lib/geometry.hh"
#include "common/common.hh"
namespace pano {

struct SSPoint;

class ExtremaDetector {
	public:
		explicit ExtremaDetector(const DOGSpace&);

		ExtremaDetector(const ExtremaDetector&) = delete;
		ExtremaDetector& operator = (const ExtremaDetector&) = delete;

		std::vector<SSPoint> get_extrema() const;

		// return extrema in global coor
		std::vector<Coor> get_raw_extrema() const;

	protected:
		const DOGSpace& dog;

		// return extrema in local coor
		std::vector<Coor> get_local_raw_extrema(int pyr_id, int scale_id) const;

		// calculate keypoint offset of a point in scalespace
		// and remove low contrast
		// See Sec.4, Accurate Keypoint Localization of Lowe,IJCV04
		bool calc_kp_offset(SSPoint* sp) const;

		std::pair<Vec, Vec> calc_kp_offset_iter(
				const DOGSpace::DOG& now_pyramid,
				int newx, int newy, int news) const;

		// Eliminating edge responses. Sec 4.1 of Lowe,IJCV04
		bool is_edge_response(Coor coor, const Mat32f& img) const;
};

}
