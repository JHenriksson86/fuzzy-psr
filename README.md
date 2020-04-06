# Fuzzy-point set registration

## Update

There is now a C++ version of the Fuzzy-PSR implemented by Qianfang. To try this approach out run the following commands.

```
roscd fuzzy-psr/data
rosrun rosrun fuzzy-psr fuzzy-psr_main
```
If you get the following error (or similar):

```
[rospack] Error: package 'fuzzy-psr' not found
```
make sure you have sourced the catkin folder where you have placed the fuzzy-psr code.


## Overview

This package contains c++ code for the fuzzy based point set registration as described by Q. Liao et al. in the paper "Point Set Registration for 3D Range Scans Using Fuzzy Cluster-based Metric and Efficient Global Optimization". There is an existing matlab implemetnation at : https://gitsvn-nt.oru.se/qianfang.liao/FuzzyClusterBasedRegistration.


## Dependencies

The idea is to have as 'limited amount of depencencies as possible'. For now ROS is even included, however, the goal is to separate ROS (and PCL) into a separate interface package later on.

For now the main depencies are:

* ceres (http://ceres-solver.org)
* Eigen
* CMake


Install ceres:
```
sudo apt install libceres-dev
```

## Matlab

The key matlab functions to be replaced are:

* fminunc (Find minimum of unconstrained multivariable function)
* ...

## Examples

Some tiny examples to test ceres / Eigen are availble in the src dir. ceres_test3.cpp utilize a score + gradient approach which looks like a place to start (unconstrainted optimization).

### Notes - useful sources etc.

* A c++ implementation of Fuzzy c-means clustering is available at : https://github.com/gyaikhom/fcm
* The ICP-Go implementation is also of interest, many functions that could be 'inspiring': https://github.com/yangjiaolong/Go-ICP
