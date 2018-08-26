// File: pybind.cc
// Author: Yuxin Wu

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "lib/mat.h"
#include "lib/imgproc.hh"
using namespace pano;


PYBIND11_PLUGIN(libpyopenpano) {
  py::module m("libpyopenpano", "docs");

  // rgb image
  py::class_<Mat32f>(m, "Mat32f").def_buffer([](Mat32f &m) -> py::buffer_info {
    return py::buffer_info(m.ptr(), // float*
                           sizeof(float),
                           py::format_descriptor<float>::format(),
                           3,
                           {(unsigned long)m.rows(), (unsigned long)m.cols(),
                            (unsigned long)m.channels()},
                           {sizeof(float) * m.cols() * m.channels(),
                            sizeof(float) * m.channels(), sizeof(float)});
  });

  // caller doesn't own the pointer
  m.def("read_img", &read_img);


  return m.ptr();
}
