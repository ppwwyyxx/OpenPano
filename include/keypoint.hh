// File: keypoint.hh
// Date: Fri Apr 19 23:22:19 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "feature.hh"
#include "sift.hh"

class KeyPoint {
	public:
		const DOGSpace& dogsp;
		const ScaleSpace& ss;
		int noctave, nscale;
		std::vector<Coor> keyp;

		std::vector<Feature> features;

		KeyPoint(const DOGSpace&, const ScaleSpace& ss);

		void detect_extrema();

		void judge_extrema(int no, int ns);

		bool judge_extrema(real_t center, int no, int ns, int i, int j);

		void get_feature(int nowo, int nows, int i, int j);

		Vec calc_offset(int, int, int, std::shared_ptr<DOG>&, real_t*, real_t*, real_t*);

		bool on_edge(int, int, const std::shared_ptr<GreyImg>&);

		void calc_dir();

		void calc_dir(Feature&, std::vector<Feature>&);

		std::vector<real_t> calc_hist(std::shared_ptr<Octave> oct, int ns, Coor coor, real_t orig_sig);

		void calc_descriptor();

		void calc_descriptor(Feature&);

		void work() {
			detect_extrema();
			calc_dir();
			calc_descriptor();
		}

};
