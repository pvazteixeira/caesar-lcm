# Caesar.jl/LCM

A C++ interface for [Caesar.jl](https://github.com/dehann/Caesar.jl) over [LCM](https://lcm-proj.github.io).

[![Build Status](https://travis-ci.org/pvazteixeira/caesar-lcm.svg?branch=master)](https://travis-ci.org/pvazteixeira/caesar-lcm)

## Dependencies

* [Eigen](https://eigen.tuxfamily.org) - `libeigen3-dev`
* [LCM](http://lcm-proj.github.io)
* [PCL](http://pointclouds.org/) - `libpcl-dev`

## Installation

Run `make` from the repository root. Executables (from `src/exes`) will be placed under `build/bin`, header files under `build/include`, and shared libraries in `build/lib`. `pkg-config` files will be in `build/lib/pkgconfig`.

## Examples

`build/bin/test` will create five poses, `x1` through `x5`, add priors (on depth, pitch and roll), connect them via local-level odometry factors (on x, y, and yaw) and add a point cloud to each pose.

## Notes
This software is constructed according to the Pods software policies and templates.  The policies and templates can be found [here](http://sourceforge.net/projects/pods).

