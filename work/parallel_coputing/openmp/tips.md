### [Configure CMakeLists](https://github.com/torbjoernk/openMP-Examples/blob/master/CMakeLists.txt)
[openmp/CMakeLists.txt at master · llvm-mirror/openmp · GitHub](https://github.com/llvm-mirror/openmp/blob/master/CMakeLists.txt)
```cmake
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") 
set(CMAKE_SHARE_LINKER_FLAGS "${CMAKE_SHARE_LINKER_FLAGS} ${OpenMP_SHARE_LINKER_FLAGS}")
endif()
```

### Example
```cpp
#ifdef _OPENMP
#include <omp.h>
#include <boost/thread.hpp>
#endif
unsigned int nthreads = boost::thread::hardware_concurrency();
omp_set_dynamic(0);
#pragma omp parallel for num_threads(nthreads)
  for (std::size_t i = 0; i < CAMERA_NUM; ++i)
  {
    // use image_v replace undistort_image_v
    cv::undistort(image_v[i]->image, undistort_image_v[i], K_mat_v_[i](cv::Rect(0, 0, 3, 3)), D_mat_v_[i]);
    pc2img_v[i] = K_mat_v_[i] * T_mat_v_[i] * pc_m; // 3*N
  }
```

+ [获取计算机核的数目用于多线程](https://stackoverflow.com/questions/150355/programmatically-find-the-number-of-cores-on-a-machine)
  - C++11
  ```cpp
    #include <thread>
    unsigned concurentThreadsSupported = std::thread::hardware_concurrency();
  ```
  - boost
  ```cpp
    #include <boost/thread.hpp>
    unsigned int nthreads = boost::thread::hardware_concurrency();
  ```
  - OpenMP
  ```cpp
  int omp_get_num_procs();
  ```