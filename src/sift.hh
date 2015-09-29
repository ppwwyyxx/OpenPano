// File: sift.hh
// Date: Fri May 03 04:50:18 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include <vector>

#include "lib/image.hh"
class Octave {
	private:
		int nscale;
		std::vector<Mat32f> data; // len = nscale
		std::vector<Mat32f> mag; // len = nscale
		std::vector<Mat32f> ort; // len = nscale, value \in [0, 2 * pi]

	public:
		int w, h;

		Octave(const Octave&) = delete;
		Octave& operator = (const Octave&) = delete;

		Octave(const Mat32f&, int num_scale);

		const Mat32f& get(int i) const {
			m_assert(i >= 0 && i < NUM_SCALE);
			return data[i];
		}

		const Mat32f& get_mag(int i) const {
			m_assert(i >= 0 && i < NUM_SCALE);
			return mag[i];
		}

		const Mat32f& get_ort(int i) const {
			m_assert(i >= 0 && i < NUM_SCALE);
			return ort[i];
		}

		int get_len() const { return nscale; }

		void cal_mag_ort(int);
};

class ScaleSpace {
	public:
		int noctave, nscale;
		int origw, origh;

		std::shared_ptr<Octave> *octaves;	// len = noctave

		ScaleSpace(const Mat32f&, int num_octave, int num_scale);

		ScaleSpace(const ScaleSpace&) = delete;
		ScaleSpace& operator = (const ScaleSpace&) = delete;

		~ScaleSpace();
};

// Calculate difference of a list of image
// diff[0] = orig[1] - orig[0]
class DOG {
	private:
		int nscale;
		std::vector<Mat32f> data;  // length is nscale - 1

	public:

		DOG(const std::shared_ptr<Octave>&);

		DOG(const DOG&) = delete;
		DOG& operator = (const DOG&) = delete;

		Mat32f diff(const Mat32f&, const Mat32f&);

		const Mat32f& get(int i) const {
			m_assert(i >= 0 && i < nscale - 1);
			return data[i];
		}

};

class DOGSpace {
	public:
		int noctave, nscale;
		int origw, origh;

		std::shared_ptr<DOG> *dogs;		// len = noctave

		DOGSpace(const DOGSpace&) = delete;
		DOGSpace& operator = (const DOGSpace&) = delete;

		DOGSpace(ScaleSpace&);

		~DOGSpace();

};
