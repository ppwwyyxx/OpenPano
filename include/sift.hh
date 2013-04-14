// File: sift.hh
// Date: Sun Apr 14 20:19:08 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <vector>
#include <memory>

#include "image.hh"
#define DESC_LEN 128

class Feature {
	public:
		int nh, nw;	// coor
		int no, ns; // octave / scale id
		int oh, ow; // coor under original scale
		int dir;
		float descriptor[DESC_LEN];
};

class Octave {
	private:
		int nscale;
		std::shared_ptr<GreyImg> *data; // len = nscale
		int w, h;

	public:
		Octave(){}
		Octave(const std::shared_ptr<GreyImg>&, int num_scale);
		Octave(const std::shared_ptr<Img>& img, int num_scale):
			Octave(std::shared_ptr<GreyImg>(new GreyImg(*img)), num_scale){ };

		const std::shared_ptr<GreyImg>& get(int i) const {
			m_assert(i >= 0 && i < NUM_SCALE);
			return data[i];
		}

		int get_len() const
		{ return nscale; }

		~Octave();

};

class ScaleSpace {
	private:
		int noctave, nscale;
	public:
		std::shared_ptr<Octave> *octaves;	// len = noctave

		ScaleSpace(const std::shared_ptr<Img>&, int num_octave, int num_scale);

		~ScaleSpace();
};

class DOG {
	private:
		int nscale;
		std::shared_ptr<GreyImg> *data;  // length is nscale - 1

	public:
		DOG(){}

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

		DOGSpace(const std::shared_ptr<Img>&, int, int);

		~DOGSpace();

};
