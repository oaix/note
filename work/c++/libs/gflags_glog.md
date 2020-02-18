[TOC]

## [Install](https://kezunlin.me/post/977f5125/)

### [gflags](https://gflags.github.io/gflags/)

去[github](https://github.com/gflags/gflags)下载最新release版本，[安装设置](https://github.com/gflags/gflags/blob/master/INSTALL.md)

```sh
cmake -DBUILD_SHARED_LIBS=ON -DINSTALL_SHARED_LIBS=ON -DINSTALL_STATIC_LIBS=OFF -DCMAKE_CONFIGURATION_TYPES=Release -DREGISTER_INSTALL_PREFIX=OFF -DNAMESPACE=google ../
# or
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make && sudo make install
```

### [glog](http://rpg.ifi.uzh.ch/docs/glog.html)

去[github](https://github.com/google/glog)下载最新release版本

```sh
cmake -DWITH_GFLAGS=ON -DCMAKE_CONFIGURATION_TYPES=Release -DBUILD_SHARED_LIBS=ON ../
make
sudo make instll


sudo rm -rf /usr/local/include/glog/
sudo rm -rf /usr/local/lib/libglog*
```

## ceres

```sh
cmake -DCXX11=ON -DGFLAGS=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF ..
```



## [CmakeLists](https://gflags.github.io/gflags/)

```cmake
find_package(gflags REQUIRED)
add_executable(foo main.cc)
target_link_libraries(foo gflags::gflags)
# or namespace = google
target_link_libraries(foo google::gflags)
```

