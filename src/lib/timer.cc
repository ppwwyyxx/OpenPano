//File: timer.cc
//Date: Fri Apr 17 15:45:45 2015 +0800
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#include "timer.hh"
#include <mutex>

// static member
std::map<std::string, double> TotalTimer::rst;

void TotalTimer::print() {
	for (auto& itr : rst)
		printf("%s spent %lf secs in total\n", itr.first.c_str(), itr.second);
}

TotalTimer::~TotalTimer() {
	static std::mutex mt;
	std::lock_guard<std::mutex> lg(mt);
	rst[msg] += timer.duration();
}
