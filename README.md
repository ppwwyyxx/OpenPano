# Panorama

## Introduction

This is an image stitching program written in C++11.

(Recently I'm working on refactoring & improvements, so there will be lots of changes)

### Compile Dependencies:

* gcc >= 4.7	(it seeems MTL doesn't work with ICC)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [CImg](http://cimg.eu/) (already included in the repository)
* [FLANN](http://www.cs.ubc.ca/research/flann/) (already included in the repository, slightly modified)
* libjpeg, libpng (you can choose what you need by commenting the macro in lib/imgio.cc)

Eigen, CImg and FLANN are header-only, to ease the compilation on different platforms.
CImg, libjpeg and libpng are only used to read and write images, so you can easily get rid of them.
You can also choose only one of libjpeg or libpng by commenting the macro in ``lib/imgio.cc``.

### Compile:
```
$ cd src; make
```

### Options:

Three modes are available (set/unset the options in ``config.cfg``):
+ __cylinder__ mode. When the following conditions meet, this mode usually yields better results:
	+ Images are taken with almost-pure single-direction rotation. (as common panoramas)
	+ Images are given in the left-to-right order. (I might fix this in the future)
	+ Images are taken with the same camera, and a good ``FOCAL_LENGTH`` is set.

+ __camera estimation mode__. No translation is the only requirement on cameras.
  It can usually work well as long as you don't have too few images.
  But it's slower because it needs to perform pairwise matches.

+ __translation mode__. Simply stitch images together by affine transformation.
  It works when camera performs pure translation.  It also requires ordered input.

Some options you may care:
+ __FOCAL_LENGTH__: focal length of your camera in [35mm equivalent](https://en.wikipedia.org/wiki/35_mm_equivalent_focal_length). Only used in cylinder mode.
+ __STRAIGHTEN__: Only used in camera estimation mode. When dealing with panorama, set this to have a more straightened result.
+ __CROP__: whether to crop the final image to avoid meaningless border.

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

In cylinder/translation mode, the input file names need to have the correct order.

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
+ Features: [SIFT](http://en.wikipedia.org/wiki/Scale-invariant_feature_transform), BRIEF
+ Transformation: use [RANSAC](http://en.wikipedia.org/wiki/RANSAC) to estimate a homography or affine transformation.
+ Optimization: focal estimation, [bundle adjustment](https://en.wikipedia.org/wiki/Bundle_adjustment), and some straightening tricks.

For details, please see [readme.pdf](https://github.com/ppwwyyxx/panorama/raw/master/readme.pdf).
