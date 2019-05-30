# OpenACC
高版本的gcc支持openacc加速，也可以使用带有该库的编译器gpi，但是在ROS环境下GCC版本不能随意升级，否则ROS无法使用。

## Config
### CmakeLists
```cmake
add_compile_options(-fopenacc)
set(CMAEK_C_FLAG "${CMAKE_C_FLAGS} -acc")
set(CMAEK_CXX_FLAG "${CMAKE_CXX_FLAGS} -acc")
```

### header file
```cpp
ifdef _OPENACC
include <openacc.h>
endif
```

### 实时查看gpu使用情况
watch -n 1 nvidia-smi # 每1秒中打印一次