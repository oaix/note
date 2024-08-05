[toc]

代码调试。

# 内存泄漏

##  [Sanitizer vs Valgrind](https://stackoverflow.com/questions/47251533/memory-address-sanitizer-vs-valgrind)

普通用户基本上只能用valgrind.



## [link release lib with debug build](https://stackoverflow.com/questions/5771935/c-linking-release-built-library-with-my-debug-build)

To link a release built library with a debug build, you need to ensure that the library is built as a DLL and that it has its own copy of the C runtime library. [You also need to avoid sharing CRT resources such as the heap across the library boundary ](https://stackoverflow.com/questions/5771935/c-linking-release-built-library-with-my-debug-build)[1](https://stackoverflow.com/questions/5771935/c-linking-release-built-library-with-my-debug-build).

[If you want to distribute a release library that others can use in either release or debug mode, you can build separate release and debug versions as static libraries ](https://stackoverflow.com/questions/5771935/c-linking-release-built-library-with-my-debug-build)[1](https://stackoverflow.com/questions/5771935/c-linking-release-built-library-with-my-debug-build).

[In CMake, you can link different libraries for debug and release builds using the `target_link_libraries` command ](https://glasshost.tech/linking-different-libraries-for-debug-and-release-builds-in-cmake-on-windows/)[2](https://glasshost.tech/linking-different-libraries-for-debug-and-release-builds-in-cmake-on-windows/).

CRT stands for C Runtime Library. CRT resources refer to resources such as the heap that are shared across the library boundary between the release and debug builds. [To link a release built library with a debug build, you need to avoid sharing CRT resources and ensure that the library is built as a DLL with its own copy of the C runtime library ](https://www.learningforjustice.org/magazine/what-critical-race-theory-is-and-what-it-means-for-teachers)

To avoid sharing CRT resources with CMake on windows, you can use the `/MT` or `/MTd` compiler flags to statically link the C runtime library into your application. [This will ensure that each binary has its own copy of the C runtime library and avoids sharing CRT resources between the release and debug builds ](https://cmake.org/cmake/help/latest/guide/tutorial/Selecting Static or Shared Libraries.html)[1](https://cmake.org/cmake/help/latest/guide/tutorial/Selecting Static or Shared Libraries.html).

To avoid sharing CRT resources with CMake on Linux, you can use the `-static-libgcc` and `-static-libstdc++` linker flags to statically link the C runtime library into your application. [This will ensure that each binary has its own copy of the C runtime library and avoids sharing CRT resources between the release and debug builds ](https://www.bing.com/aclick?ld=e8mKU8qo7Wa612Q1NzcLEmqTVUCUxJ_LNBvz07sUfBGHYjsVOBpvKhQ6dD1BXXsqQhGxTZq7kmULQ2iVOXICftO62U_WGGm5FEErUB-k1VvbBbBJMWor6TmsoiTej96Wmt4snVwKFlbhxndbEoWd-hEWm5tT3xTQrrRI8cu1lRVFW4cPmD&u=aHR0cHMlM2ElMmYlMmZ3d3cuc3VzZS5jb20lMmYlM2Z1cmwlM2RodHRwcyUyNTNBJTI1MkYlMjUyRnd3dy5zdXNlLmNvbSUyNTJGbGliZXJ0eS1saW51eCUyNTNGdXRtX3NvdXJjZSUyNTNEYmluZyUyNTI2dXRtX21lZGl1bSUyNTNEcGFpZHNlYXJjaCUyNTI2dXRtX2NhbXBhaWduJTI1M0QxMF9GWTIzUTRfR0RHX0VMX0xMX1BETFhfT0FfTGVnYWN5X0hvc3RhZ2VfQmluZ19SR19HTEJfbXBfMjAyNDMzNiUyNTI2dXRtX2NvbnRlbnQlMjUzRHRleHRfZW4lMjZfYnQlM2QlMjZfYmslM2RyZWQlMjUyMGhhdCUyNTIwbGludXglMjZfYm0lM2RiJTI2X2JuJTNkbyUyNl9iZyUzZDEzMTUwMTgyMjc5OTUwMTAlMjZtc2Nsa2lkJTNkZmNjMzEzZWNjMTExMTM4ZDQwMzQ3NmI5MTExM2Q5YjQ&rlid=fcc313ecc111138d403476b91113d9b4)[1](https://www.baeldung.com/linux/set-command)

```cmake
if(CMAKE_COMPILER_IS_GNUCXX)
    # Use static runtime libraries
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -static-libgcc -static-libstdc++")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -static-libgcc -static-libstdc++")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -static-libgcc -static-libstdc++")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -static-libgcc -static-libstdc++")
endif()

if(MSVC)
    # Use static runtime libraries
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif()

```

