

[TOC]



### [图标desktop](https://stackoverflow.com/questions/14674425/exec-run-two-commands)

https://www.ibm.com/developerworks/cn/linux/l-cn-dtef/index.html

https://developer.gnome.org/desktop-entry-spec/

<https://askubuntu.com/questions/281293/creating-a-desktop-file-for-a-new-application>

```
[Desktop Entry]
Name=振镜增益标定
Name[zh_CN]=振镜增益标定
Comment=mems_scanning_angle
Exec= sh -c "/usr/bin/mems_scanning_angle 2>&1 | tee -a /$HOME/output.log"
Icon=/usr/share/icons/mems_scanning_angle.png
Terminal=true
Type=Application
Categories=Application;
Encoding=UTF-8
StartupNotify=true
```

执行多条指令：

```sh
Exec=sh -c "command 1 ; command 2"
```



### [CHANGELOG](https://keepachangelog.com/zh-CN/0.3.0/)

每一个软件的版本必须：

+ 标明日期（要用上面说过的规范）
+ 标明分类（采用英文）。规范如下：
+ 'Added' 添加的新功能
+ 'Changed' 功能变更
+ 'Deprecated' 不建议使用，未来会删掉
+ 'Removed' 之前不建议使用的功能，这次真的删掉了
+ 'Fixed' 改的bug
+ 'Security' 改的有关安全相关bug

请参考例子[CHANGELOG](./CHANGELOG.md)