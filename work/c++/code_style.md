[TOC]

## function

### parameters

+ 输入参数使用值或者const 引用，输出参数使用指针pointer；输入参数可以使用`const pointers`，但是绝不使用`non-const reference`，除非特殊函数例如swap()。
+ However, there are some instances where using const T* is preferable to const T& for input parameters. For example:    
  - You want to pass in a null pointer.
  - The function saves a pointer or reference to the input.

## [clang-format](http://startheap.com/2018/08/17/The-method-by-which-vim-configures-clang-format-under-Linux/)

```sh
clang-format -style=file -i `find . -type f -regex ".*\.\(cpp\|h\)"`
```

