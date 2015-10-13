# Panorama

## Introduction

This is an image stitching program written in C++11.

(Recently I'm working on refactoring & improvements, so there will be lots of changes)

### Compile Dependencies:

* gcc >= 4.7	(it seeems MTL doesn't work with ICC)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [CImg](http://cimg.eu/) (already included in the repository)
* libjpeg, libpng (you can choose what you need by commenting the macro in lib/imgio.cc)

Both Eigen and CImg are header-only, to ease the compilation on different platforms.
CImg, libjpeg and libpng are only used to read and write images, so you can easily get rid of them.
You can also choose only one of libjpeg or libpng by commenting the macro in ``lib/imgio.cc``.

### Compile:
```
$ cd src; make
```

### Options:

Two modes are available (set/unset the option ``CYLINDER`` in ``config.cfg``):
+ __cylinder__ mode. When the following conditions meet, this mode usually yields better results:
	+ Images are taken with almost-pure rotation. (as common panoramas)
	+ Images are given in the correct order. (I might fix this in the future)
	+ Images are taken with the same camera, and a good FOCAL_LENGTH is set.

+ __general__ mode. This mode has no assumptions on input images. So it'll be slower.

Some options you may care:
+ __FOCAL_LENGTH__: focal length of your camera in [35mm equivalent](https://en.wikipedia.org/wiki/35_mm_equivalent_focal_length). Only used in cylinder mode.
+ __STRAIGHTEN__: Only used in general mode. Whether to try straighten the result. Good to set when dealing with panorama.
+ __CROP__: whether to crop the final image to avoid black border.

Other parameters are quality-related.
The default values are generally good for images with more than 0.7 megapixels.
If your images are too small and cannot give satisfactory results,
it might be better to resize your images rather than tune the parameters.

### Run:

```
$ ./image-stitching <file1> <file2> ...
```

The output file is ``out.png``.

Before dealing with very large images (4 megapixels or more), it's better to resize them. (I might add this feature in the future)

In cylinder mode, the input file names need to have the correct order.

## Examples:

Zijing Apartment in Tsinghua University:
![dorm](https://github.com/ppwwyyxx/panorama/raw/master/results/small/apartment.jpg)

Myselves:
![myself](https://github.com/ppwwyyxx/panorama/raw/master/results/small/myself.jpg)

Zijing Playground in Tsinghua University:
![planet](https://github.com/ppwwyyxx/panorama/raw/master/results/small/planet.jpg)

Carnegie Mellon University
![cmu0](https://github.com/ppwwyyxx/panorama/raw/master/results/small/CMU0-all.jpg)
![apple](https://github.com/ppwwyyxx/panorama/raw/master/results/apple.jpg)



For more examples, see [results](https://github.com/ppwwyyxx/panorama/tree/master/results).

## Algorithms
I use [SIFT](http://en.wikipedia.org/wiki/Scale-invariant_feature_transform) for feature detection and [RANSAC](http://en.wikipedia.org/wiki/RANSAC) to estimate transformation.

For more details, please see [readme.pdf](https://github.com/ppwwyyxx/panorama/raw/master/readme.pdf).
