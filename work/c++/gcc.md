[TOC]

# GCC

## [Options for Directory Search](https://gcc.gnu.org/onlinedocs/gcc/Directory-Options.html)
These options specify directories to search for header files, for libraries and for parts of the compiler. 
If dir begins with ‘=’ or $SYSROOT, then the ‘=’ or $SYSROOT is replaced by the sysroot prefix; see --sysroot and -isysroot.
#include <>直接到系统指定的某些目录中去找某些头文件。
#include “”先到源文件所在文件夹去找，然后再到系统指定的某些目录中去找某些头文件。
+ `-I` dir
  Add the directory dir to the list of directories to be searched for header files.  Directories named by -I are searched before the standard system include directories. 如果跟的dir是系统路径，那么忽略此条设置，后面系统路径自然会按顺序搜索。同时搜索`#inlcude "file"` 和`#include <file>`
  [gcc -I Explained](https://www.cleancss.com/explain-command/gcc/5227)
+ `-iquote dir`
  #inlcude "file"在该路径下搜索，`#include <file>`不在该路径下搜索，且该路径的搜索顺序在所有`-I`路径和系统路径之前
+ `-isystem dir`
  Search dir for header files, after all directories specified by -I but before the standard system directories. 
+ `idirafter dir`

## [update](https://gist.github.com/application2000/73fd6f4bf1be6600a2cf9f56315a2d91)

```sh
GCC 8.1.0 on Ubuntu 14.04 & 16.04 & 18.04:

sudo apt-get update -y && 
sudo apt-get install build-essential software-properties-common -y && 
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y && 
sudo apt-get update -y && 
sudo apt-get install gcc-8 g++-8 -y && 
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-8 && 
sudo update-alternatives --config gcc

select gcc-8

# sudo add-apt-repository ppa:jonathonf/gcc-8.0 
# sudo apt-get update
# sudo apt-get install gcc-8
```



## [#ifndef vs #pragma once](https://stackoverflow.com/questions/1143936/pragma-once-vs-include-guards)

```cpp
#pragma once
#ifndef _HEADER_H_
#define _HEADER_H_

...

#endif
```

`#pragma once`虽然能够加速编译，但是有些编译器不支持，并且无法区分`xx/a.h`,`yy/a.h`，有潜在风险。可以像上面那样同时使用`#ifndef`和`#pragma once`。可以使用自动生成工具`VAssistX`来自动添加到头文件。

To be on the safe side, you should still prefer the classical header guards -- #pragma once is not C++ standard, and no compiler is forced to support it.



## [gcc常规操作](https://www.cnblogs.com/testlife007/p/6555404.html)

从程序员的角度看，函数库实际上就是一些头文件（.h）和库文件（.so 或 .a）的集合。虽然Linux下的大多数函数都默认将头文件放到 /usr/include/ 目录下，而库文件则放到 /usr/lib/ 目录下，但并不是所有的情况都是这样。正因如此，gcc 在编译时必须有自己的办法来查找所需要的头文件和库文件。常用的方法有：
**(1) -I** 
可以向 gcc 的头文件搜索路径中添加新的目录。
**(2) -L** 
如果使用了不在标准位置的库文件，那么可以通过 -L 选项向 gcc 的库文件搜索路径中添加新的目录。
**(3) -l** 
Linux下的库文件在命名时有一个约定，就是应该以 lib 这3个字母开头，由于所有的库文件都遵循了同样的规范，因此在用 -l 选项指定链接的库文件名时可以省去 lib 这3个字母。例如，gcc 在对 -lfoo 进行处理时，会自动去链接名为 libfoo.so 的文件。
**(4) -static**
Linux下的库文件分为两大类，分别是：动态链接库（通常以 .so 结尾）和静态链接库（通常以 .a 结尾）。
两者的差别仅在程序执行时所需的代码是在运行时动态加载的，还是在编译时静态加载的。
默认情况下，gcc 在链接时优先使用动态链接库，只有当动态链接库不存在时才考虑使用静态链接库。
如果需要的话，可以在编译时加上 -static 选项，强制使用静态链接库。
**(5) -shared**
生成一个共享的目标文件，它能够与其他的目标一起链接生成一个可执行的文件。