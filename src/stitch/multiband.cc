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
	add_images.emplace_back(ImageToAdd{Range{upper_left, bottom_right}, img, coor_func});
	target_size.update_max(bottom_right);
}

void MultiBandBlender::create_first_level() {
	GuardedTimer tm("MultiBandBlender::create_first_level()");

	int nr_image = add_images.size();
	image_metas.reserve(nr_image);
	REP(k, nr_image) {
		ImageToAdd& img = add_images[k];
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
				wimg.at(i, j).w = std::max(
						(0.5f - fabs(orig_coor.x)) * (0.5f - fabs(orig_coor.y)),
						0.0) + EPS;
				// ext? eps?
			}
		}
		img.imgref.release();
		image_metas.emplace_back(ImageMeta{range, move(mask)});
		images.emplace_back(ImageToBlend{move(wimg), image_metas.back()});
	}
	add_images.clear();
}

Mat32f MultiBandBlender::run() {
	GuardedTimer tm("MultiBandBlender::run()");
	create_first_level();
	update_weight_map();
	Mat32f target(target_size.y, target_size.x, 3);
	fill(target, Color::NO);

	Mask2D target_mask(target_size.y, target_size.x);

	for (auto& m : images)
		next_lvl_images.emplace_back(m);
	for (int level = 0; level < band_level; level ++) {
		GuardedTimer tmm("Blending level " + to_string(level));
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
				m_assert(ccur.get_min() >= 0);
				auto & img_next = next_lvl_images[imgid];
				auto& cnext = img_next.color_on_target(j, i);
				m_assert(cnext.get_min() >= 0);

				// TODO special for last
				isum += (ccur - cnext) * w;
				wsum += w;
			}
			if (wsum < EPS)
				continue;
			isum /= wsum;
			float* p = target.ptr(i, j);
			if (not target_mask.get(i, j)) {		// first time to visit *p. Note that *p could be negative after visit.
				isum.write_to(p);
#pragma omp critical	// omp walks through rows. optimize?
				target_mask.set(i, j);
			} else {
				p[0] += isum.x, p[1] += isum.y, p[2] += isum.z;
			}
		}
		swap(next_lvl_images, images);
	}

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
	REP(i, target_size.y) REP(j, target_size.x) {
		float max = 0.f;
		for (auto& img : images)
			if (img.meta.range.contain(i, j))
				update_max(max, img.weight_on_target(j, i));
		if (max == 0.f)
			continue;
		max -= EPS*0.5f;
		for (auto& img : images) if (img.meta.range.contain(i, j)) {
			float& w = img.weight_on_target(j, i);
			//w = w / max;
			w = (w >= max);
			if (w == 1)
				max += 1;		// avoid setting two weight both to 1
		}
	}
}

void MultiBandBlender::create_next_level(int level) {
	TotalTimer tm("create_next_level()");
	m_assert(next_lvl_images.size() == images.size());
	if (level == band_level - 1) {
#pragma omp parallel for schedule(dynamic)
		REP(k, images.size()) {
			auto& img = images[k].img;
			// don't have to do this when band_level > 1
			auto& wimg = (next_lvl_images[k].img = Mat<WeightedPixel>(img.rows(), img.cols(), 1));
			memset(wimg.ptr(), 0, sizeof(WeightedPixel) * img.rows() * img.cols());
		}
	} else {
		GaussianBlur blurer(sqrt(level * 2 + 1.0) * 4);	// TODO size
#pragma omp parallel for schedule(dynamic)
		REP(i, images.size()) {
			next_lvl_images[i].img = blurer.blur(images[i].img);
		}
	}
}


}	// namespace pano
