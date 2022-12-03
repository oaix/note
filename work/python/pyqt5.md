### [c++调用pyqt5](<https://www.coder.work/article/1729768>)

c++的qt版本需要和pyqt5的版本兼容，不然无法实现调用。

<https://nitratine.net/blog/post/how-to-import-a-pyqt5-ui-file-in-a-python-gui/>

<http://www.360doc.com/content/19/0515/15/9824753_835884983.shtml>

<https://github.com/eyllanesc/stackoverflow/tree/master/questions/55685163>

```cmake
find_package(PythonLibs 3 REQUIRED)
include_directories(/usr/include/python3.5)
target_link_libraries(
  ${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::Network
  ${OpenCV_LIBRARIES}
  MVSDK
  rs_mems_scanning_angle_calib
  phase_auto_calib
  logger
  ${PYTHON_LIBRARIES})

```

+ [调用python类](<https://blog.csdn.net/sihai12345/article/details/82745350>)

  <https://cloud.tencent.com/developer/article/1174629>

