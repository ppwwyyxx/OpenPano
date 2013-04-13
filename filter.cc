// File: filter.cc
// Date: Thu Apr 11 13:36:59 2013 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "filter.hh"
#include "utils.hh"
#include "render/MImageRender.hh"
using namespace std;

shared_ptr<GreyImg> Filter::GaussianBlur(const shared_ptr<GreyImg> img, real_t sigma) {
	print_debug("gaussian with sigma %lf\n", sigma);
	HWTimer timer;
	int w = img->w, h = img->h;
	shared_ptr<GreyImg> ret(new GreyImg(*img));

	const int kw = ceil(6 * sigma);
	const int center = kw / 2;

	real_t normalization_factor = 2 * M_PI * sqr(sigma);
	real_t kernel_tot_all = 0;

	real_t ** kernel = new real_t*[kw];
	for (int i = 0; i < kw; i ++) {
		kernel[i] = new real_t[kw];
		for (int j = 0; j < kw; j ++) {
			real_t x = i - center,
				   y = j - center;
			kernel[i][j] = exp(-(sqr(x) + sqr(y)) / (2 * sqr(sigma)));
			kernel[i][j] /= normalization_factor;
			kernel_tot_all += kernel[i][j];
		}
	}

	for (int i = 0; i < h; i ++)
		for (int j = 0; j < w; j ++) {
			int x_bound = min(kw, h + center - i), y_bound = min(kw, w + center - j);
			real_t kernel_tot = 0;
			if (j >= center && x_bound == kw && i >= center && y_bound == kw)
				kernel_tot = kernel_tot_all;
			else {
				for (int x = max(center - i, 0); x < x_bound; x ++)
					for (int y = max(center - j, 0); y < y_bound; y ++)
						kernel_tot += kernel[x][y];
			}

			real_t compensation = 1.0 / kernel_tot;
			real_t newvalue = 0;
			for (int x = max(0, center - i); x < x_bound; x ++)
				for (int y = max(0, center - j); y < y_bound; y ++) {
					int dj = y - center + j,
						di = x - center + i;
					real_t curr = img->get_pixel(di, dj);
					newvalue += curr * (kernel[x][y] * compensation);
				}
			ret->set_pixel(i, j, newvalue);
		}
	print_debug("sec: %lf\n", timer.get_sec());
	return move(ret);
}

shared_ptr<GreyImg> Filter::GreyScale(const shared_ptr<Img> img) {
	shared_ptr<GreyImg> ret(new GreyImg(*img));
	return move(ret);
}

real_t Filter::to_grey(const ::Color& c)
{ return 0.299 * c.x + 0.587 * c.y + 0.114 * c.z; }
