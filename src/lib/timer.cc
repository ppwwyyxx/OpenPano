//File: timer.cc
//Date: Fri Apr 17 15:45:45 2015 +0800
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "timer.hh"
#include "debugutils.hh"
#include <mutex>

// static member
std::map<std::string, std::pair<int, double>> TotalTimer::rst;

void TotalTimer::print() {
	for (auto& itr : rst)
		print_debug("%s spent %lf secs in total, called %d times.\n",
				itr.first.c_str(), itr.second.second, itr.second.first);
}

TotalTimer::~TotalTimer() {
	static std::mutex mt;
	std::lock_guard<std::mutex> lg(mt);
	auto& p = rst[msg];
	p.second += timer.duration();
	p.first ++;
}
