# Panorama

## Introduction

This is an image stitching program written in C++11.

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

Without special needs, we only have to set ``PANO``, ``TRANS``, ``CROP``.

The program does some extra work to optimize the output
when knowing the input pictures were taken by a camera moving in pure translation or pure rotation.

* ``PANO = 1`` indicates that the camera moved in pure rotation. A panorama is expected to be the output;
* ``TRANS = 1`` indicates that the camera moved in pure translation. Result will be better than one without this option set;
* ``CROP`` decides whether to crop the final image to a rectangular;
* NOTE that PANO and TRANS can not be both set.

### Run:

```
$ ./main <file1> <file2> ...
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
