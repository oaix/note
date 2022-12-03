[TOC]

# bugs
### QT designer 设计的UI，生成对应的部件头文件

+ 使用命令输出头文件

  ```sh
  /usr/lib/x86_64-linux-gnu/qt5/bin/uic -o ui_mainwindow.h mainwindow.ui 
  ```

+ [直接在CMakeLists中配置](https://www.freehacker.cn/tools/cmake-create-qt5-project/)

  - 带有`Q_OBJECT`宏的头文件，需要被`moc`程序处理，CMake提供了[QT5_WRAP_CPP](https://doc.qt.io/qt-5/qtcore-cmake-qt5-wrap-cpp.html)命令来生成对应的文件，并保存在`${CMAKE_CURRENT_BINARY_DIR}`。
  - `.ui`界面设计文件，需要被`uic`程序处理，CMake提供了`QT5_WRAP_UI`命令，该命令用来生成ui文件对应的头文件，生成的头文件在`${CMAKE_CURRENT_BINARY_DIR}`。
  - `.qrc`资源文件，需要被`rcc`程序处理

  `QT5_WRAP_CPP`和`QT5_WRAP_UI`会在编译路径`${CMAKE_CURRENT_BINARY_DIR}`下生成相应的文件，因此我们需要将`${CMAKE_CURRENT_BINARY_DIR}`设置到包含路径中：

  ```cmake
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  ```



  https://cmake.org/cmake/help/latest/manual/cmake-qt.7.html#automoc

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

### [QObject::connect: Cannot queue arguments of type 'string'](https://stackoverflow.com/questions/18477498/qt-own-enum-in-class)

在main函数或者class的构造函数中：`qRegisterMetaType<string>("string");`注册该类型



### [获取网口名称，IP](https://blog.csdn.net/caoshangpa/article/details/51073138)

https://blog.csdn.net/wjs1033/article/details/22697063

```cpp
QString address = "192.168.1.200";
QHostAddress mems_address(address);
    QList<QNetworkInterface> networkInterface = QNetworkInterface::allInterfaces();
    for (QList<QNetworkInterface>::const_iterator it = networkInterface.constBegin(); it != networkInterface.constEnd();
         ++it)
    {
      QList<QNetworkAddressEntry> address_entry_list = (*it).addressEntries();
      for (auto address_entry = address_entry_list.begin(); address_entry != address_entry_list.end(); ++address_entry)
      {
        if (address_entry->ip().protocol() == QAbstractSocket::IPv4Protocol &&
            address_entry->ip().isInSubnet(mems_address, 24))
        {
          net_name_ = (*it).humanReadableName();
          // qDebug() << address_entry->ip().toString();
          // qDebug() << address_entry->netmask().toString();
        }
      }
    }
```



### [settings](https://www.jianshu.com/p/bebde752908f)

```cpp
QSettings settings("RoboSense", "MEMS_B2_Scan_Scope");
  if (QFile(settings.fileName()).exists())
  {
    restoreGeometry(settings.value("geometry").toByteArray());

    ui_->doubleSpinBoxCalibDist->setValue(settings.value("calibDist").toDouble());
    ui_->spinBoxWidthAngularPoint->setValue(settings.value("widthAngularPoint").toInt());
    ui_->spinBoxHeightAngularPoint->setValue(settings.value("heightAngularPoint").toInt());
    ui_->doubleSpinBoxGridLength->setValue(settings.value("gridLength").toDouble());
    ui_->lineEditDataSaveAddr->setText(settings.value("dataSaveAddr").toString());
    ui_->doubleSpinBoxThreshold->setValue(settings.value("threshold").toDouble());
    qDebug() << "Read settings successfully!";
  }
  else
  {
    qDebug() << "Fail to read settings. Config file doesn't exist, please update configuration and save it.";
  }
```



### [QTimer](https://blog.csdn.net/jiezhj/article/details/31770837)

https://zhuanlan.zhihu.com/p/33697117

+ 执行一次

  ```cpp
  // 10s后将相机设置为手动曝光
  QTimer::singleShot(10000, this, SLOT(cancelAutoExposure()));
  ```

+ 间隔执行

  ```cpp
  temperature_timer_ = new QTimer(this);
      QObject::connect(temperature_timer_, SIGNAL(timeout()), this, SLOT(slotShowTemperature()));
      temperature_timer_->start(2000);  //  ms
  ```



### [QThread销毁](https://blog.csdn.net/newyher/article/details/53190067)

https://www.thinbug.com/q/25224575

```cpp
capture_thread_ = new CaptureThread(this);  //相机抓取线程
QObject::connect(capture_thread_, SIGNAL(finished()), capture_thread_, SLOT(deleteLater()));
//开启
if (!capture_thread_->isRunning())
    {
      capture_thread_->start();
    }
    capture_thread_->stream();
//销毁
if (capture_thread_ != Q_NULLPTR && capture_thread_->isRunning())
  {
    capture_thread_->stop();
    capture_thread_->quit();
    capture_thread_->wait();
    delete capture_thread_;
  
```

```cpp
// Thread.hpp
#include <QThread>
public Thread : class QThread {
  Q_OBJECT
  using QThread::run; // This is a final class
public:
  Thread(QObject * parent = 0);
  ~Thread();
}

// Thread.cpp
#include "Thread.h"
Thread::Thread(QObject * parent): QThread(parent)
{}

Thread::~Thread() {
  quit();
  #if QT_VERSION >= QT_VERSION_CHECK(5,2,0)
  requestInterruption();
  #endif
  wait(); 
}
```

### [QThread使用注意点](https://www.kdab.com/the-eight-rules-of-multithreaded-qt/)

### [use QTimer in thread](<http://blog.debao.me/2013/08/how-to-use-qthread-in-the-right-way-part-1/>)

<https://stackoverflow.com/questions/3794649/qt-events-and-signal-slots/3794944#3794944>

<https://www.cnblogs.com/liushui-sky/p/5829563.html>

<https://doc.qt.io/qt-5/threads-qobject.html>

+ QThread中signal与main thread中槽函数使用Qt::QueuedConnection连接
+ main thread中signal不要连接QThread中的slot，QThread的所有slot应该申明为private

[thread-basics](https://het.as.utexas.edu/HET/Software/html/thread-basics.html)

https://het.as.utexas.edu/HET/Software/html/threads-qobject.html#signals-and-slots-across-threads

### [check qtcpsocket connected](<https://stackoverflow.com/questions/10445122/qtcpsocket-state-always-connected-even-unplugging-ethernet-wire/10447560#10447560>)

每个1秒读取mems温度，在其槽函数中开启另外一个QTimer t1，时间可以是2s，在tcp的readyRead()槽函数中重置t1，如果长期没有进入该槽函数，说明tcp连接异常。

### sleep

```c++
void sleepMilliSeconds(const unsigned int msec)
{
  QEventLoop loop;
  QTimer::singleShot(msec, &loop, SLOT(quit()));
  loop.exec();
}

// 如果没有事件，怎会消耗cpu
void sleepMilliSeconds(const unsigned int msec)
{
  QTime dieTime = QTime::currentTime().addMSecs(msec);
  while (QTime::currentTime() < dieTime)
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
```

