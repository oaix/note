# Qt
1. QT designer 设计的UI，生成对应的部件头文件：
    /usr/lib/x86_64-linux-gnu/qt5/bin/uic -o ui_mainwindow.h mainwindow.ui 
2. 更改之后，以前的部件名称仍然在.ui文件当中，此时运行上面命名时会有警告
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

