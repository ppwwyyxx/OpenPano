// File: keypoint.hh
// Date: Fri May 03 01:37:46 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "feature.hh"
#include "lib/geometry.hh"
#include "dog.hh"

// This should be renamed to SIFTPoint
// Descriptor should only have coor & featvec
class SIFTPoint {
	public:
		Coor coor;
		Vec2D real_coor;
		int no, ns; // octave / scale id
		real_t dir;
		real_t scale_factor;

		real_t descriptor[DESC_LEN];

		SIFTPoint(){}

		SIFTPoint(const SIFTPoint& r):
			coor(r.coor), real_coor(r.real_coor), no(r.no), ns(r.ns), dir(r.dir),
			scale_factor(r.scale_factor){
			memcpy(descriptor, r.descriptor, DESC_LEN * sizeof(real_t));
		}

		Descriptor to_descriptor() const {
			Descriptor ret;
			REP(i, DESC_LEN)
				ret.descriptor.emplace_back(descriptor[i]);
			ret.coor = real_coor;
			return ret;
		}

};

class KeyPoint {
	public:
		std::vector<Coor> keyp;

		KeyPoint(const DOGSpace&, const ScaleSpace& ss);

		void work() {
			detect_feature();
			calc_dir();
			calc_sift_descriptor();
		}

		std::vector<Descriptor> get_sift_descriptor() const;

	protected:
		const DOGSpace& dogsp;
		const ScaleSpace& ss;
		int noctave, nscale;

		std::vector<SIFTPoint> features;

		void detect_feature();

		void judge_extrema(int no, int ns);

		bool judge_extrema(real_t center, int no, int ns, int i, int j);

		void get_feature(int nowo, int nows, int i, int j);

		Vec calc_offset(int, int, int,
				const DOGSpace::DOG &, real_t*, real_t*, real_t*);

		bool on_edge(int, int, const Mat32f&);

		void calc_dir();

		void calc_dir(SIFTPoint&, std::vector<SIFTPoint>&);

		std::vector<real_t> calc_hist(
				const GaussianPyramid& oct,
				int ns, Coor coor, real_t orig_sig);

		void calc_sift_descriptor();

		// calculate and write to the input
		void calc_sift_descriptor(SIFTPoint&);

};
