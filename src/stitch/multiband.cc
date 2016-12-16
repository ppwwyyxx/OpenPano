//File: multiband.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "multiband.hh"
#include "lib/imgproc.hh"
#include "feature/gaussian.hh"

using namespace std;
namespace pano {
void MultiBandBlender::add_image(
			const Coor& upper_left,
			const Coor& bottom_right,
			ImageRef &img,
			std::function<Vec2D(Coor)> coor_func) {
	images_to_add.emplace_back(ImageToAdd{Range{upper_left, bottom_right}, img, coor_func});
	target_size.update_max(bottom_right);
}

void MultiBandBlender::create_first_level() {
	GUARDED_FUNC_TIMER;

	int nr_image = images_to_add.size();
	meta_images.reserve(nr_image);	// we will need reference to this vector element
#pragma omp parallel for schedule(dynamic)
	REP(k, nr_image) {
		ImageToAdd& img = images_to_add[k];
		img.imgref.load();

		auto& range = img.range;
		Mat<WeightedPixel> wimg(range.height(), range.width(), 1);
		Mask2D mask(range.height(), range.width());
		REP(i, range.height()) REP(j, range.width()) {
			Coor target_coor{j + range.min.x, i + range.min.y};
			Vec2D orig_coor = img.coor_func(target_coor);
			Color c = interpolate(*img.imgref.img, orig_coor.y, orig_coor.x);
			if (c.get_min() < 0) {	// Color::NO
				wimg.at(i, j).w = 0;
				wimg.at(i, j).c = Color::BLACK;	// -1 will mess up with gaussian blur
				mask.set(i, j);
			} else {
				wimg.at(i, j).c = c;
				orig_coor.x = orig_coor.x / img.imgref.width() - 0.5;
				orig_coor.y = orig_coor.y / img.imgref.height() - 0.5;
				wimg.at(i, j).w = std::max(0.0,
						(0.5f - fabs(orig_coor.x)) * (0.5f - fabs(orig_coor.y))) + EPS;
				// ext? eps?
			}
		}
		img.imgref.release();
#pragma omp critical
		{
			meta_images.emplace_back(MetaImage{range, move(mask)});
			images.emplace_back(ImageToBlend{move(wimg), meta_images.back()});
		}
	}
	images_to_add.clear();
}

Mat32f MultiBandBlender::run() {
	create_first_level();
	update_weight_map();
	Mat32f target(target_size.y, target_size.x, 3);
	fill(target, Color::NO);

	Mask2D target_mask(target_size.y, target_size.x);

	for (auto& m : images)
		next_lvl_images.emplace_back(m);
	for (int level = 0; level < band_level; level ++) {
		bool is_last = (level == band_level - 1);
		GuardedTimer tmm("Blending level " + to_string(level));
		if (!is_last)
			create_next_level(level);
		//debug_level(level);
#pragma omp parallel for schedule(dynamic)
		REP(i, target_size.y) REP(j, target_size.x) {
			Color isum(0, 0, 0);
			float wsum = 0;
			REP(imgid, images.size())  {
				auto& img_cur = images[imgid];
				if (not img_cur.meta.range.contain(i, j)) continue;
				if (not img_cur.valid_on_target(j, i)) continue;

				float w = img_cur.weight_on_target(j, i);
				if (w <= 0) continue;

				auto& ccur = img_cur.color_on_target(j, i);

				if (not is_last) {
					auto & img_next = next_lvl_images[imgid];
					auto& cnext = img_next.color_on_target(j, i);
					isum += (ccur - cnext) * w;
				} else {
					isum += ccur * w;
				}
				wsum += w;
			}
			if (wsum < EPS)
				continue;
			isum /= wsum;
			float* p = target.ptr(i, j);
			if (not target_mask.get(i, j)) {		// first time to visit *p. Note that *p could be negative after visit.
				isum.write_to(p);
				target_mask.set(i, j);
			} else {
				p[0] += isum.x, p[1] += isum.y, p[2] += isum.z;
			}
		}
		swap(next_lvl_images, images);
	}
	images.clear(); next_lvl_images.clear();

	REP(i, target.rows()) REP(j, target.cols()) {
		if (target_mask.get(i, j)) {
			float* p = target.ptr(i, j);
			// weighted laplacian pyramid might introduce minor over/under flow
			p[0] = max(min(p[0], 1.0f), 0.f);
			p[1] = max(min(p[1], 1.0f), 0.f);
			p[2] = max(min(p[2], 1.0f), 0.f);
		}
	}
	return target;
}

void MultiBandBlender::update_weight_map() {
	GUARDED_FUNC_TIMER;
#pragma omp parallel for schedule(dynamic, 100)
	REP(i, target_size.y) REP(j, target_size.x) {
		float max = 0.f;
		float* maxp = nullptr;
		for (auto& img : images) {
			if (img.meta.range.contain(i, j)) {
				float& w = img.weight_on_target(j, i);
				if (w > max) {
					max = w;
					maxp = &w;
				}
				w = 0;
			}
		}
		if (maxp) *maxp = 1;
	}
}

void MultiBandBlender::create_next_level(int level) {
	TOTAL_FUNC_TIMER;
	GaussianBlur blurer(sqrt(level * 2 + 1.0) * 4);	// TODO size
#pragma omp parallel for schedule(dynamic)
	REP(i, (int)images.size())
		next_lvl_images[i].img = blurer.blur(images[i].img);
}


}	// namespace pano
