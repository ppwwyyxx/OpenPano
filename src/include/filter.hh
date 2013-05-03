// File: filter.hh
// Date: Sat May 04 01:33:12 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <memory>
#include "image.hh"

class GaussCache {
	public:
		real_t ** kernel;
		real_t kernel_tot;
		real_t normalization_factor;

		int kw;

		GaussCache(real_t sigma);

		~GaussCache()
		{ free_2d<real_t>(kernel, kw); }

		// something bad
		GaussCache(const GaussCache& m) {
			kernel_tot = m.kernel_tot;
			normalization_factor = m.normalization_factor;
			kw = m.kw;
			kernel = new real_t* [kw];
			REP(i, kw) {
				kernel[i] = new real_t[kw]();
				memcpy(kernel[i], m.kernel[i], kw * sizeof(real_t));
			}
		}

		GaussCache & operator = (const GaussCache & m) {
			if (this != &m) {
				kernel_tot = m.kernel_tot;
				normalization_factor = m.normalization_factor;
				kw = m.kw;
				kernel = new real_t* [kw];
				REP(i, kw) {
					kernel[i] = new real_t[kw]();
					memcpy(kernel[i], m.kernel[i], kw * sizeof(real_t));
				}
			}
			return *this;
		}

		GaussCache & operator = (GaussCache && r) {
			m_assert(this != &r);
			free_2d<real_t>(kernel, kw);
			kernel_tot = r.kernel_tot;
			normalization_factor = r.normalization_factor;
			kw = r.kw;
			kernel = r.kernel;
			r.kernel = nullptr;
			return *this;
		}

		GaussCache(GaussCache&& m) {
			kernel_tot = m.kernel_tot;
			normalization_factor = m.normalization_factor;
			kw = m.kw;
			kernel = m.kernel;
			m.kernel = nullptr;
		}

};

class Filter {
	public:
		std::vector<GaussCache> gcache;		// size = nscale - 1
		Filter(int nscale, real_t gauss_sigma, real_t scale_factor);

		std::shared_ptr<GreyImg> GaussianBlur(const std::shared_ptr<GreyImg>&, const GaussCache&) const;
		std::shared_ptr<GreyImg> GaussianBlur(const std::shared_ptr<GreyImg>& img, int n) const
		{return GaussianBlur(img, gcache[n - 1]); }
		static std::shared_ptr<GreyImg> GreyScale(const std::shared_ptr<Img>);
		static real_t to_grey(const ::Color&);
};
