// File: dog.hh
// Date: Fri May 03 04:50:18 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include "lib/mat.h"
#include "lib/debugutils.hh"
#include "lib/config.hh"
#include "common/common.hh"
namespace pano {

// Given an image, build an octave with different blurred version
class GaussianPyramid {
	private:
		int nscale;
		std::vector<Mat32f> data; // len = nscale
		std::vector<Mat32f> mag; // len = nscale
		std::vector<Mat32f> ort; // len = nscale, value \in [0, 2 * pi]

		void cal_mag_ort(int);

	public:
		int w, h;

		GaussianPyramid(const Mat32f&, int num_scale);

		inline const Mat32f& get(int i) const { return data[i]; }

		inline const Mat32f& get_mag(int i) const { return mag[i]; }

		inline const Mat32f& get_ort(int i) const { return ort[i]; }

		int get_len() const { return nscale; }
};

class ScaleSpace {
	public:
		int noctave, nscale;
		const int origw, origh;

		std::vector<GaussianPyramid> pyramids;	// len = noctave

		ScaleSpace(const Mat32f&, int num_octave, int num_scale);

		ScaleSpace(const ScaleSpace&) = delete;
		ScaleSpace& operator = (const ScaleSpace&) = delete;
};


class DOGSpace {

	public:
		// Calculate difference of a list of image
		// diff[0] = orig[1] - orig[0]

		typedef std::vector<Mat32f> DOG;	// len = nscale - 1

		int noctave, nscale;
		int origw, origh;

		std::vector<DOG> dogs;		// len = noctave

		DOGSpace(const DOGSpace&) = delete;
		DOGSpace& operator = (const DOGSpace&) = delete;

		Mat32f diff(const Mat32f& img1, const Mat32f& img2) const;
		DOGSpace(ScaleSpace&);

};

}
