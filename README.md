# Panorama

## Introduction

This is an image stitching program written in C++11.

(Recently I'm working on refactoring & improvements, so there will be lots of changes)

### Compile Dependencies:

* gcc >= 4.7	(it seeems MTL doesn't work with ICC)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [CImg](http://cimg.eu/) (already included in the repository)
* libjpeg, libpng (choose what you need by commenting the macro in lib/imgproc.cc)

Both Eigen and CImg are header-only, so ideally you can easily compile the code on any platform.

### Compile:
```
$ make
```

### Config:

Various parameters are saved in ``src/config.cfg``

Without special needs, you only have to set ``CROP``, which decides whether to crop the final image to a rectangular;

### Run:

```
$ ./image-stitching <file1> <file2> ...
```

The output file is ``out.png``.

Usually, input images should not exeeds 20(files) x 1500(pixels) x 1000(pixels), since your computer may not have enough memory.

The input file names given in the command line need to be well ordered, to make sure the adjacent images can be stitched together.

## Examples:

Zijing Apartment in Tsinghua University:
![dorm](https://github.com/ppwwyyxx/panorama/raw/master/results/small/apartment.jpg)

Myselves:
![myself](https://github.com/ppwwyyxx/panorama/raw/master/results/small/myself.jpg)

Zijing Playground in Tsinghua University:
![planet](https://github.com/ppwwyyxx/panorama/raw/master/results/small/planet.jpg)

For more examples, see [results](https://github.com/ppwwyyxx/panorama/tree/master/results).

## Algorithms
I use [SIFT](http://en.wikipedia.org/wiki/Scale-invariant_feature_transform) for feature detection and [RANSAC](http://en.wikipedia.org/wiki/RANSAC) to estimate transformation.

For more details, please see [readme.pdf](https://github.com/ppwwyyxx/panorama/raw/master/readme.pdf).
