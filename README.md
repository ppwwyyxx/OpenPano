# Panorama

## Introduction

This is an image stitching program written in C++11.

### Compile Dependencies:

* gcc >= 4.7
* [Magick++](http://www.imagemagick.org/Magick++/)
* [Boost MTL](http://www.simunova.com/node/145)

### Compile:
```
$ make
```

### Config:

Various parameters are saved in ``src/config.cfg``

Without special needs, we only have to modify ``PANO``, ``TRANS``, ``CROP``.

The program does some extra work to beautify the output if knowing the input pictures were taken by a camera moving in pure translation or pure rotation.

* ``PANO = 1`` tells that the camera moved in pure rotation. A panorama is expected to be the output;
* ``TRANS = 1`` tells that the camera moved in pure translation. Result will be better than the one with this option unset;
* ``CROP`` decides whether to crop the final image to a rectangular;

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

For more details, please see [report.pdf](https://github.com/ppwwyyxx/panorama/raw/master/report.pdf).
