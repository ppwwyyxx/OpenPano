# OpenPano

![cmu0](results/CMU0-all.jpg)

## Introduction

OpenPano is a panorama stitching program written in C++ from scratch (without any vision libraries). It mainly follows the routine
described in the paper [Automatic Panoramic Image Stitching using Invariant Features](http://matthewalunbrown.com/papers/ijcv2007.pdf),
which is also the one used by [AutoStitch](http://matthewalunbrown.com/autostitch/autostitch.html).

## Installation

We need the following dependencies:
* gcc >= 5, clang >= 10 or visual studio >= 2015. CMake >= 3.20
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* libjpeg (optional, if you only need png format)
* [FLANN](http://www.cs.ubc.ca/research/flann/) (already included in the repository, slightly modified)
* [CImg](http://cimg.eu/) (optional. already included in the repository)

Eigen, CImg and FLANN are header-only, to simplify the compilation on different platforms.
CImg and libjpeg are only used to read and write images, so you can easily get rid of them.

On ArchLinux, install dependencies by: `sudo pacman -S gcc sed cmake make libjpeg eigen`

On Ubuntu, install dependencies by: `sudo apt install build-essential sed cmake libjpeg-dev libeigen3-dev`

### Compile:
#### Linux / OSX / WSL (bash on windows)
Use cmake (a good default to try):
```
$ cmake -B build && make -C build
# Binary will be found at ./build/src/image-stitching
```
or, use make (more customizable. You can modify Makefile when you run into problems.):
```
$ make -C src
# Binary will be found at ./src/image-stitching
```
The default clang on OSX doesn't contain openmp support.
You may need gcc or different clang. See [#16](https://github.com/ppwwyyxx/OpenPano/issues/16).


#### Windows (for VS2015)
* Install cmake, VS2015
* Set environment variable `Eigen3_DIR` to `{YOUR_EIGEN3_DIRECTORY}/eigen3/cmake`.
* Open Visual Studio [Developer Command Prompt](https://msdn.microsoft.com/en-us/library/ms229859(v=vs.110).aspx).
* `cd path/to/OpenPano`
* `cmake .`
* Open the VS2015 project and compile the project
* copy `config.cfg` to the directory containing `image-stitching.exe`
* The author have never used Visual Studio and this windows build process may not work for you. Feel
	free to submit PR to improve the build process.

## Usage

```
$ ./image-stitching <file1> <file2> ...
```

The output file is ``out.jpg``. You can play with the [example data](https://github.com/ppwwyyxx/OpenPano/releases/tag/0.1) to start with.

Before dealing with very large images (4 megapixels or more), it's better to manually downscale them to save time.

In cylinder/translation mode, the input file names need to have the correct order.

### Configuration:

The program expects to find the config file `config.cfg` in the working directory.
Three modes are available (set/unset them in the top of the config file):
+ __cylinder__ mode. Requirements:
	+ You stay at the same spot and __only__ turn left (or right) when taking the images (as is usually done), no
		translations or other type of rotations allowed.
	+ Images are taken with the same camera, with a known ``FOCAL_LENGTH`` set in config.
	+ Images are given in the left-to-right order. (I might fix this in the future)

+ __camera estimation mode__. Requirements:
  * You stay at the same spot when taking the images, and can turn your camera left-right or
    up-down.
  * Don't use too few images.
  * It runs slower because it needs to perform pairwise matches.

+ __translation mode__. Simply stitch images together by affine transformation. Requirements:
  * Camera performs pure translation.
  * The images are roughly at the same depth.
  * Input images are ordered according to the translation movement.

Some options you may care:
+ __FOCAL_LENGTH__: focal length of your camera in [35mm equivalent](https://en.wikipedia.org/wiki/35_mm_equivalent_focal_length). Only useful in cylinder mode.
+ __ORDERED_INPUT__: whether input images are ordered sequentially. has to be `1` in CYLINDER and TRANS mode.
+ __CROP__: whether to crop the final image to avoid irregular white border.

Other parameters are quality-related.
The default values are generally good for images with more than 0.7 megapixels.
If your images are too small and cannot produce satisfactory results,
it might be better to resize your images rather than tune the parameters.


## Examples ([All original data available for __download__](https://github.com/ppwwyyxx/OpenPano/releases/tag/0.1))

Zijing Apartment in Tsinghua University:
![dorm](results/apartment.jpg)

"Myselves":
![myself](results/myself.jpg)

<!--
   -Zijing Playground in Tsinghua University:
   -![planet](https://github.com/ppwwyyxx/panorama/raw/master/results/planet.jpg)
	 -->

Carnegie Mellon University from 38 images
![apple](results/apple.jpg)

Newell-Simon Hall in CMU (this one is hard because objects are closer):
![nsh](results/NSH-all.jpg)

A full-view pano built from UAV images:
![uav](results/uav.jpg)

For more examples, see [results](results).

## Speed & Memory
Tested on Intel Core i7-6700HQ, with `ESTIMATE_CAMERA` mode:

+ 11 ordered images of size 600x400: 3.2s.
+ 13 ordered images of size 1500x1112: 6s.
+ 38 unordered images of size 1300x867 (high vertical FOV): 51s.

Memory consumption is known to be huge with default libc allocator.
Simply using a modern allocator (e.g. tcmalloc, jemalloc) can help a lot.
Also, setting `LAZY_READ` to 1 can save memory at the cost of a minor slow down.

Peak memory in bytes (assume each input has the same w & h):

+ Without `LAZY_READ` option: max(finalw \* finalh \* 12, #photos \* w \* h \* 12 + #photos \* #matched\_pairs * 96 + #keypoints * 520)
+ With `LAZY_READ` option: max(finalw \* finalh \* 16, #threads \* w \* h \* 12, #photos \* #matched\_pairs * 96 + #keypoints * 520)

## Algorithms
+ Features: [SIFT](http://en.wikipedia.org/wiki/Scale-invariant_feature_transform)
+ Transformation: use [RANSAC](http://en.wikipedia.org/wiki/RANSAC) to estimate a homography or affine transformation.
+ Optimization: focal estimation, [bundle adjustment](https://en.wikipedia.org/wiki/Bundle_adjustment), and some straightening tricks.

For details, see [my blog post](http://ppwwyyxx.com/blog/2016/How-to-Write-a-Panorama-Stitcher/).

## Quality Guidelines

To get the best stitching quality:
+ While rotating the camera for different shots, try to keep the position of camera lens static.
+ Keep the exposure parameters unchanged.
+ Do not shoot on moving objects.
+ Objects far away will stitch better.
+ The algorithm doesn't work well with wide-angle cameras where images are distorted heavily. Camera
	parameters are needed to undistort the images.

## TODOs
+ Github Actions for macOS and Windows
+ apply pairwise matching for translation mode as well
+ run bundle adjustment on sphere lens instead of perspective lens
+ improve feature detector and matching
+ use LAZY_READ & 1 byte image in both blender to reduce peak memory
+ clean up use of copies of `ImageRef`
+ faster gaussian blur kernel
+ port some hotspot (e.g. `dist.cc`) to neon
+ support read/write EXIF metadata to:
	+ get focal length, distortion, etc
	+ allow pano to be viewed on Facebook
+ python bindings
