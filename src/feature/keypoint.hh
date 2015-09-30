// File: keypoint.hh
// Date: Fri May 03 01:37:46 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "feature.hh"
#include "lib/geometry.hh"
#include "dog.hh"

class KeyPoint {
	public:
		std::vector<SIFTFeature> features;
		std::vector<Coor> keyp;

		KeyPoint(const DOGSpace&, const ScaleSpace& ss);

		void work() {
			detect_feature();
			calc_dir();
			calc_descriptor();
		}

	protected:
		const DOGSpace& dogsp;
		const ScaleSpace& ss;
		int noctave, nscale;


		void detect_feature();

		void judge_extrema(int no, int ns);

		bool judge_extrema(real_t center, int no, int ns, int i, int j);

		void get_feature(int nowo, int nows, int i, int j);

		Vec calc_offset(int, int, int,
				const DOGSpace::DOG &, real_t*, real_t*, real_t*);

		bool on_edge(int, int, const Mat32f&);

		void calc_dir();

		void calc_dir(SIFTFeature&, std::vector<SIFTFeature>&);

		std::vector<real_t> calc_hist(
				const Octave& oct,
				int ns, Coor coor, real_t orig_sig);

		void calc_descriptor();

		// calculate and write to the input
		void calc_descriptor(SIFTFeature&);

};
