//File: debug.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "blender.hh"
#include "multiband.hh"
#include "stitcher.hh"
#include "match_info.hh"

#include <iostream>

#include "lib/utils.hh"
#include "lib/imgproc.hh"
#include "lib/planedrawer.hh"

using namespace std;

namespace pano {

void LinearBlender::debug_run(int w, int h) {
#pragma omp parallel for schedule(dynamic)
	REP(k, (int)images.size()) {
		auto& img = images[k];
		img.imgref.load();
		Mat32f target(h, w, 3);
		fill(target, Color::NO);
		for (int i = 0; i < target.height(); i ++) {
			float *row = target.ptr(i);
			for (int j = 0; j < target.width(); j ++) {
				Color isum = Color::BLACK;
				if (img.range.contain(i, j)) {
					Vec2D img_coor = img.map_coor(i, j);
					if (!img_coor.isNaN()) {
						float r = img_coor.y, c = img_coor.x;
						isum = interpolate(*img.imgref.img, r, c);
					}
				}
				isum.write_to(row + j * 3);
			}
		}
		print_debug("Debug rendering %02d image\n", k);
		write_rgb(ssprintf("log/blend-%02d.jpg", k), target);
	}
}

void MultiBandBlender::debug_level(int level) const {
	int imgid = 0;
	// TODO omp
	for (auto& t: images) {
		auto& wimg = t.img;
		Mat32f img(wimg.rows(), wimg.cols(), 3);
		Mat32f weight(wimg.rows(), wimg.cols(), 3);
		REP(i, wimg.rows()) REP(j, wimg.cols()) {
			if (not t.meta.mask.get(i, j))
				wimg.at(i, j).c.write_to(img.ptr(i, j));
			else
				Color::NO.write_to(img.ptr(i, j));
			float* p = weight.ptr(i, j);
			p[0] = p[1] = p[2] = wimg.at(i, j).w;
		}
		print_debug("[MultiBand] debug output image %d\n", imgid);
		write_rgb(ssprintf("log/multiband%d-%d.jpg", imgid, level), img);
		write_rgb(ssprintf("log/multibandw%d-%d.jpg", imgid, level), weight);
		imgid ++;
	}
}


void Stitcher::draw_matchinfo() {
	int n = imgs.size();
	REP(i, n) imgs[i].load();
#pragma omp parallel for schedule(dynamic)
	REP(i, n) REPL(j, i+1, n) {
		Vec2D offset1(imgs[i].width()/2, imgs[i].height()/2);
		Vec2D offset2(imgs[j].width()/2 + imgs[i].width(), imgs[j].height()/2);
		Shape2D shape2{imgs[j].width(), imgs[j].height()},
						shape1{imgs[i].width(), imgs[i].height()};

		auto& m = pairwise_matches[i][j];
		if (m.confidence <= 0)
			continue;
		list<Mat32f> imagelist{*imgs[i].img, *imgs[j].img};
		Mat32f conc = hconcat(imagelist);
		PlaneDrawer pld(conc);
		for (auto& p : m.match) {
			pld.set_rand_color();
			pld.circle(p.first + offset1, 7);
			pld.circle(p.second + offset2, 7);
			pld.line(p.first + offset1, p.second + offset2);
		}

		pld.set_color(Color(0,0,0));

		Matrix homo(3,3);
		REP(i, 9) homo.ptr()[i] = m.homo[i];
		Homography inv = m.homo.inverse();
		auto p = overlap_region(shape1, shape2, homo, inv);
		for (auto& v: p) v += offset1;
		pld.polygon(p);

		Matrix invM(3, 3);
		REP(i, 9) invM.ptr()[i] = inv[i];
		p = overlap_region(shape2, shape1, invM, m.homo);
		for (auto& v: p) v += offset2;
		pld.polygon(p);

		print_debug("Dump matchinfo of %d->%d\n", i, j);
		write_rgb(ssprintf("log/match%d-%d.jpg", i, j), conc);
	}
}

void Stitcher::dump_matchinfo(const char* fname) const {
	print_debug("Dump matchinfo to %s\n", fname);
	ofstream fout(fname);
	m_assert(fout.good());
	int n = imgs.size();
	REP(i, n) REP(j, n) {
		auto& m = pairwise_matches[i][j];
		if (m.confidence <= 0) continue;
		fout << i << " " << j << endl;
		m.serialize(fout);
		fout << endl;
	}
	fout.close();
}

void Stitcher::load_matchinfo(const char* fname) {
	print_debug("Load matchinfo from %s\n", fname);
	ifstream fin(fname);
	int i, j;
	int n = imgs.size();
	pairwise_matches.resize(n);
	for (auto& k : pairwise_matches) k.resize(n);

	while (true) {
		fin >> i >> j;
		if (fin.eof()) break;
		pairwise_matches[i][j] = MatchInfo::deserialize(fin);
	}
	fin.close();
}

}
