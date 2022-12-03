# [gtest](https://github.com/google/googletest)

https://google.github.io/googletest/primer.html

https://google.github.io/googletest/

[参考例子](https://github.com/kaizouman/gtest-cmake-example)

```cmake
cmake_minimum_required(VERSION 2.6)
 
# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
# Link runTests with what we want to test and the GTest and pthread library
add_executable(runTests tests.cpp)
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)
```

https://www.cnblogs.com/coderzh/archive/2009/04/06/1426755.html

[install on Ubuntu20.04](https://zwarrior.medium.com/install-google-test-framework-gtest-on-ubuntu-20-04-368eb6951b12)

```sh
sudo apt  install libgtest-dev
cd /usr/src/gtest
sudo make
sudo cp ./lib/libgtest*.a /usr/lib
```

