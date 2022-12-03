[TOC]



### [Can't link program using Boost.Filesystem](https://stackoverflow.com/questions/15634114/cant-link-program-using-boost-filesystem/17988317)

在程序中包含的相关Boost头文件中禁止C ++ 11范围内的枚举:

```cpp
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
```

