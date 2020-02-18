### [cuda内核中使用__ldg时出错(Error using __ldg in cuda kernel at compile time)](http://www.it1352.com/491089.html)

`__ldg()`仅适用于计算能力3.5（或更高版本）架构。

+ 必须在计算3.5（或更新版本）GPU上运行

+ 必须针对计算3.5（或更新版本）的GPU进行编译

  这不起作用： nvcc -arch = sm_30 ...

+ 不能为旧体系结构编译



### [DotProductCuda.cpp:27: error: expected primary-expression before `'<'` token](https://stackoverflow.com/questions/5986070/error-compiling-cuda-expected-primary-expression)

`nvcc` uses the file extension to determine how to process the contents of the file. If you have CUDA syntax inside the file, it must have a .cu extension, otherwise nvcc will simply pass the file untouched to the host compiler, resulting in the syntax error you are observing.

调用kernel函数的函数必须包含在*.cu文件中，nvcc识别.cu文件。头文件后缀`.cuh`,否则使用系统编译器进行编译。

### [全局常量内存必须和调用的函数处于一个文件](http://bbs.gpuworld.cn/index.php?topic=10801.0)

cudaMemcpyToSymbol的调用和你定义的常量内存，必须在同一个文件才能使用。当然，使用这个常量内存的global函数也必须在这个文件才可以。这是CUDA的规定。



### [nvcc fatal: A single input file is required for a non-link phase when an outputfile is specified](https://answers.ros.org/question/251156/unable-to-solve-nvcc-fatal-a-single-input-file-is-required-for-a-non-link-phase-when-an-outputfile-is-specified-error/)

vtk的macros括号两侧有空格符，Due to the fact that nvcc can't handle function-like macros with spaces around the parenthesis, an error is given.　As a solution, one can clear all the compiler flags (thus getting rid of the redundant function-like macros due to VTK) by using the ``set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )`` command in cmake before building cuda with the CUDA_COMPILE cmake macro.

1. add 'set_directory_properties(` PROPERTIES COMPILE_DEFINITIONS "" ) after cxx/nvcc flags setting
2. or to remove 'add_definitions(${PCL_DEFINITIONS})

在CmakeLists.txt中添加如下：

```cmake
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )
```

也可以[清除rtk相关的定义](https://github.com/PointCloudLibrary/pcl/issues/776)：

```cmake
get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()
```

