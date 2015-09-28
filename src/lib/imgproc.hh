//File: imgproc.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "mat.h"

Mat32f read_rgb(const char* fname);

template <typename T>
void resize(const Mat<T> &src, Mat<T> &dst);
