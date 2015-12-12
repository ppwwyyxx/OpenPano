//File: debug.cc
//Author: Yuxin Wu <ppwwyyxx@gmail.com>

#include "blender.hh"
#include "stitcher.hh"
#include "match_info.hh"

#include <iostream>

#include "lib/utils.hh"
#include "lib/imgproc.hh"
#include "lib/planedrawer.hh"

namespace pano {

void LinearBlender::debug_run(int w, int h) {
#pragma omp parallel for schedule(dynamic)
	REP(k, (int)images.size()) {
		auto& img = images[k];
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
						if (!is_edge_color(img.img, r, c))
							isum = interpolate(img.img, r, c);
					}
				}
				isum.write_to(row + j * 3);
			}
		}
		print_debug("Debug rendering %02d image\n", k);
		write_rgb(ssprintf("log/blend-%02d.jpg", k).c_str(), target);
	}
}


void Stitcher::draw_matchinfo() const {
	int n = imgs.size();
	REP(i, n) REPL(j, i+1, n) {
		auto& m = pairwise_matches[j][i];
		if (m.confidence <= 0) continue;
		print_debug("Dump matchinfo of %d->%d\n", i, j);
		list<Mat32f> imagelist{imgs[i], imgs[j]};
		Mat32f conc = vconcat(imagelist);
		PlaneDrawer pld(conc);
		for (auto& p : m.match) {
			pld.set_rand_color();
			Coor icoor1 = Coor(p.second.x + imgs[i].width()/2,
					p.second.y + imgs[i].height()/2);
			Coor icoor2 = Coor(p.first.x + imgs[j].width()/2,
					p.first.y + imgs[j].height()/2);
			pld.circle(icoor1, 7);
			pld.circle(icoor2 + Coor(0, imgs[i].height()), 7);
			pld.line(icoor1, icoor2 + Coor(0, imgs[i].height()));
		}
		write_rgb(ssprintf("log/match%d-%d.jpg", i, j).c_str(), conc);
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
