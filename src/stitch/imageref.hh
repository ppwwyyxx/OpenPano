//File: imageref.hh
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <string>
#include <memory>
#include "lib/mat.h"
#include "lib/imgproc.hh"
#include "match_info.hh"
#include "common/common.hh"
namespace pano {
// A transparent reference to a image in file
struct ImageRef {
  std::string fname;
  Mat32f* img = nullptr;
  int _width, _height;

  ImageRef(const std::string& fname): fname(fname) {}
  //ImageRef(const ImageRef& ) = delete;  // TODO make it work
  ~ImageRef() { if (img) delete img; }

  void load() {
    if (img) return;
    img = new Mat32f{read_img(fname.c_str())};
    _width = img->width();
    _height = img->height();
  }

  void release() { if (img) delete img; img = nullptr; }

  int width() const { return _width; }
  int height() const { return _height; }
  Shape2D shape() const { return {_width, _height}; }

};

}
