# [GDB](https://www.jianshu.com/p/719b2e626080)

详细使用可参考[官方文档](https://sourceware.org/gdb/current/onlinedocs/gdb/)。编译时加上-g参数，保留调试信息，否则不能使用GDB进行调试。常用[指令请查看](https://www.jianshu.com/p/b7896e9afeb7)。

gdb可[配合使用sanitizer](https://www.jianshu.com/p/3a2df9b7c353)。https://github.com/google/sanitizers/wiki/AddressSanitizerAndDebugger

https://zhuanlan.zhihu.com/p/74897601

https://www.yanbinghu.com/2018/09/26/61877.html

https://blog.csdn.net/weiwangchao_/article/details/11882617

## 1 [生产core文件](https://blog.csdn.net/wteruiycbqqvwt/article/details/103630076)

*ulimit -c unlimited   # 取消core文件大小限制*

core文件默认的存储位置与对应的可执行程序在同一目录下，文件名是core。

`echo "/corefile/core-%e-%p-%t" > core_pattern` # 可以将core文件统一生成到/corefile目录下，产生的文件名为core-命令名-pid-时间戳.通过core查看问题：

```sh
gdb a.out core # where / breaktrace(bt)
```

