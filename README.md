# TSAM
## 1. Introduction
TSAM is a library of C++ classes that implement smoothing and mapping (SAM) in robotics and vision, using factor graphs and Bayes networks as the underlying computing paradigm rather than sparse matrices.

## 2. Compile
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=ON -DEIGEN_DONT_VECTORIZE=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_SHARED_LIBS=OFF  -DGTSAM_BUILD_TESTS=ON -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_INSTALL_PREFIX=../out
make -j
make install
```
Tips: Android源码依赖，编译静态库，动态库有些符合没导出，可能会链接错误。
