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

