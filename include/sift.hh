// File: sift.hh
// Date: Wed May 01 10:27:13 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <memory>

#include "image.hh"
// TODO: add mag and ori in Octave data
class Octave {
	private:
		int nscale;
		std::shared_ptr<GreyImg> *data; // len = nscale
		std::shared_ptr<GreyImg> *mag; // len = nscale
		std::shared_ptr<GreyImg> *ort; // len = nscale, value \in [0, 2 * pi]

	public:
		int w, h;

		Octave(const std::shared_ptr<GreyImg>&, int num_scale);

		Octave(const std::shared_ptr<Img>& img, int num_scale):
			Octave(std::shared_ptr<GreyImg>(new GreyImg(*img)), num_scale){ };

		const std::shared_ptr<GreyImg>& get(int i) const {
			m_assert(i >= 0 && i < NUM_SCALE);
			return data[i];
		}

		const std::shared_ptr<GreyImg>& get_mag(int i) const {
			m_assert(i >= 0 && i < NUM_SCALE);
			return mag[i];
		}

		const std::shared_ptr<GreyImg>& get_ort(int i) const {
			m_assert(i >= 0 && i < NUM_SCALE);
			return ort[i];
		}

		int get_len() const
		{ return nscale; }

		void cal_mag_ort(int);

		~Octave();

};

class ScaleSpace {
	public:
		int noctave, nscale;
		int origw, origh;

		std::shared_ptr<Octave> *octaves;	// len = noctave

		ScaleSpace(const std::shared_ptr<Img>&, int num_octave, int num_scale);

		~ScaleSpace();
};

class DOG {		// diff[0] = orig[1] - orig[0]
	private:
		int nscale;
		std::shared_ptr<GreyImg> *data;  // length is nscale - 1

	public:

		DOG(const std::shared_ptr<Octave>&);

		~DOG();

		std::shared_ptr<GreyImg> diff(const std::shared_ptr<GreyImg>&, const std::shared_ptr<GreyImg>&);

		const std::shared_ptr<GreyImg>& get(int i) const {
			m_assert(i >= 0 && i < nscale - 1);
			return data[i];
		}

};

class DOGSpace {
	public:
		int noctave, nscale;
		int origw, origh;

		std::shared_ptr<DOG> *dogs;		// len = noctave

		DOGSpace(ScaleSpace&);

		~DOGSpace();

};
