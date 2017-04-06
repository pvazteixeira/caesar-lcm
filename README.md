# RoME-LCM

A C++ interface for [RoME](https://github.com/dehann/rome.jl) over [LCM](https://lcm-proj.github.io).

## Dependencies

* [Eigen]() - `libeigen3-dev`
* [LCM]() - 
* [PCL]() - `libpcl-dev`

## Installation

Run `make` from the repository root. Executables (from `src/exes`) will be placed under `build/bin`, header files under `build/include`, and shared libraries in `build/lib`. `pkg-config` files will be in `build/lib/pkgconfig`.

## Examples

`build/bin/test` will create two poses, `x1` and `x2`, add priors (on depth, pitch and roll), connect them with an odometry factor (on x, y, and yaw) and add a point cloud to each.

## Notes
This software is constructed according to the Pods software policies and
templates.  The policies and templates can be found [here](http://sourceforge.net/projects/pods).

