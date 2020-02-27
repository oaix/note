[toc]

# bugs
### QT designer 设计的UI，生成对应的部件头文件

+ 使用命令输出头文件

  ```sh
  /usr/lib/x86_64-linux-gnu/qt5/bin/uic -o ui_mainwindow.h mainwindow.ui 
  ```

+ 直接在CMakeLists中配置

  ```cmake
  set(QT_FORMS ui/main_window.ui ui/config_dialog.ui)
  set(HEADER_FILES src/main_window.h src/config_dialog.h src/capture_thread.h)
  qt5_wrap_ui(UIC_FORMS ${QT_FORMS})#直接生产头文件到编译地址
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})# 处理QT Object头文件
  ```

更改之后，以前的部件名称仍然在.ui文件当中，此时运行上面命名时会有警告
mainwindow.ui: Warning: Z-order assignment: 'layoutWidget' is not a valid widget.
mainwindow.ui: Warning: Z-order assignment: 'horizontalSpacer' is not a valid widget.
此时用编译器打开.ui 文件，然后删除`<zorder>xxxx</zorder>`

```
<zorder>layoutWidget</zorder>
<zorder>layoutWidget</zorder>
<zorder>layoutWidget</zorder>
<zorder>label_18</zorder>
<zorder>progressBar</zorder>
<zorder>horizontalSpacer</zorder>
```



### [QMetaObject::connectSlotsByName: No matching signal for](https://blog.csdn.net/u012997311/article/details/51313992)

按空间名称关联槽”的方式进行关 联，对应的函数必须写成`on_控件名_信号名`的格式；或者也可以通过connet函数人为显式地将信号和槽关联起来。但是，如果采用显式 connect的方法的同时，又将槽函数的名字起成了``on_控件名_信号名`的格式，那么就会在运行时弹出 “QMetaObject::connectSlotsByName: No matching signal for”的警告了！





