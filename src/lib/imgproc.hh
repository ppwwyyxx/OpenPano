//File: imgproc.hh
//Date:
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include "mat.h"
#include "color.hh"

// forward declaration
class Matrix;

#include <list>
Mat32f read_rgb(const char* fname);
void write_rgb(const char* fname, const Mat32f& mat);

Mat32f hconcat(const std::list<Mat32f>& mats);

Color interpolate(const Mat32f& mat, float r, float c);

Mat32f crop(const Mat32f& mat);

Mat32f rgb2grey(const Mat32f& mat);

// get transform from p2 to p1
Matrix getPerspectiveTransform(const std::vector<Vec2D>& p1, const std::vector<Vec2D>& p2);
Matrix getAffineTransform(const std::vector<Vec2D>& p1, const std::vector<Vec2D>& p2);

// judge Color::NO
bool is_edge_color(const Mat32f& mat, float y, float x);

void fill(Mat32f& mat, const Color& c);

template <typename T>
void resize(const Mat<T> &src, Mat<T> &dst);
