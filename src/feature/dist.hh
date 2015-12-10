//File: dist.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <limits>
#include "lib/debugutils.hh"

namespace pano {

float euclidean_sqr(
		const float* x, const float* y,
		size_t n, float now_thres);

int hamming(const float* x, const float* y, int n);

// a L2 implementation compatible with FLANN to use
// work for float array of size 4k
struct L2SSE {
    typedef bool is_kdtree_distance;
    typedef float ElementType;
    typedef float ResultType;

    template <typename Iterator1, typename Iterator2>
    inline float operator()(
				Iterator1 a, Iterator2 b,
				size_t size, ResultType worst_dist = -1) const {
				if (worst_dist <= 0) worst_dist = std::numeric_limits<float>::max();
				return pano::euclidean_sqr(a, b, size, worst_dist);
    }

    template <typename U, typename V>
    inline ResultType accum_dist(const U& a, const V& b, int) const {
        return (a-b)*(a-b);
    }
};

}
