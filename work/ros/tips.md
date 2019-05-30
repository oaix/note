# Tips
1. ROS函数输出参数使用指针传递，而不是引用，Output arguments to methods / functions (i.e., variables that the function can modify) are passed by pointer, not by reference；
  如果是引用传递，　调用者在不看函数原型的情况下，无法分辨该参数是否可以被修改。输入参数通常使用值或者const reference.

2. catkin package
  For a package to be considered a catkin package it must meet a few requirements:
   + The package must contain a catkin compliant package.xml file. This package.xml file provides meta information about the package.
   + The package must contain a CMakeLists.txt which uses catkin. Catkin metapackages must have a boilerplate CMakeLists.txt file.
   + There can be no more than one package in each folder. This means no nested packages nor multiple packages sharing the same directory

3. [catkin_EXPORTED_TARGETS](http://docs.ros.org/jade/api/catkin/html/howto/format2/cpp_msg_dependencies.html)
  For C++ access to ROS messages, CMake needs to find the message or service headers:
```cmake
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs)
include_directories(include ${catkin_INCLUDE_DIRS}) 
```
The include parameter is needed only if that subdirectory of your package contains headers also needed to compile your programs.
Since you presumably have build targets using the message or service headers, add this to ensure all their headers get built before any targets that need them:
```cmake
add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
add_dependencies(your_library ${catkin_EXPORTED_TARGETS})
```
Example:
```cmake
add_rostest_gtest(test_system_node system_node.test system.cpp)
add_dependencies(test_system_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_system_node ${catkin_LIBRARIES})
```

4. nodelet
  http://wiki.ros.org/nodelet
  http://wiki.ros.org/nodelet/Tutorials/Running%20a%20nodelet
  http://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html
  https://blog.csdn.net/yiranhaiziqi/article/details/53308657

5. [metapackage](http://wiki.ros.org/catkin/package.xml#Metapackages)
  A good use for metapackages is to group the major components of your robot and then provide a comprehensive grouping for your whole system.
  group multiple packages as a single logical package, Catkin packages must depend directly on the packages they use, not on any metapackages. but metapackages can depend on other metapackages. metapackage可以依赖其他metapackage, 普通package不能依赖于matepackage.
+ CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(velodyne)
find_package(catkin REQUIRED)
catkin_metapackage()
```
+ package.xml
```xml
<package format="2">
  <name>velodyne</name>
  <version>1.3.0</version>
  <description>
    Basic ROS support for the Velodyne 3D LIDARs.
  </description>
  <maintainer email="jwhitley@autonomoustuff.com">Josh Whitley</maintainer>
  <author>Jack O'Quin</author>
  <license>BSD</license>
  <url type="website">http://www.ros.org/wiki/velodyne</url>
  <url type="repository">https://github.com/ros-drivers/velodyne</url>
  <url type="bugtracker">https://github.com/ros-drivers/velodyne/issues</url>

  <buildtool_depend>catkin</buildtool_depend>

  <exec_depend>velodyne_driver</exec_depend>
  <exec_depend>velodyne_laserscan</exec_depend>
  <exec_depend>velodyne_msgs</exec_depend>
  <exec_depend>velodyne_pointcloud</exec_depend>

  <export>
    <metapackage/>
  </export>
</package>
```
Because the metapackage **CMakeLists.txt** contains a catkin macro, its package.xml must declare a buildtool dependency on catkin:
`<buildtool_depend>catkin</buildtool_depend>`
Additional buildtool, build or test dependencies are not permitted.
Metapackages list all packages or other metapackages in their group using <run_depend> tags.

6. `cv_bridge::CvImage`的编码可以是`opencv`中的类型
  `sensor_msgs::image_encodings::TYPE_32FC1`

7. `sensor_msgs::Image`可以有２种publish方式
+ ros::Publisher
```cpp
ros::Publisher pub = nh_.advertise<sensor_msgs::Image>("/color_mat", 10);
```
+ image_transport::ImageTransport
```cpp
image_transport::ImageTransport it(nh);
image_transport::Publisher img_intenstiy_pub = it.advertise("img_intensity", 10);
```

8. ros 同步`message_filters::sync_policies::ApproximateTime`
  `<depend>message_filters</depend>`
```cpp
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
message_filters::Subscriber<sensor_msgs::Image> img_sub_[CAMERA_NUM];
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image>
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
pc_sub_.subscribe(nh_, pc_sub_, 10);
        img_sub_.subscribe(nh_, img_sub_, 10);
sync_.reset(new Sync(MySyncPolicy(10), pc_sub_, img_sub_[0], img_sub_[1], img_sub_[2], img_sub_[3], img_sub_[4]));
  sync_->registerCallback(boost::bind(&SyncPointCloudImages::callback, this, _1, _2, _3, _4, _5, _6));
```

9. cmakelists查看对应的文件是否存在
```
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/bumblebee.launch)
  roslaunch_add_file_check(launch/camera.launch)
  roslaunch_add_file_check(launch/stereo.launch)

  find_package(roslint REQUIRED)
  roslint_cpp()
endif()
```

10. 重新定义`ctl + c`
```cpp
SyncImagePointCloud* g_sync_img_pc;
void mySigintHandler(int sig)
{
  g_sync_img_pc->stopReceiveData();
  g_sync_img_pc->syncImagePointCloud();
  std::cout << "before ros node shut down" << std::endl;
  ros::shutdown();
  std::cout << "after ros node shut down" << std::endl;
}
  ros::init(argc, argv, "sync_five_cameras_pc_offline", ros::init_options::NoSigintHandler);

```

11. [callback signature](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)
    合法的回调信号
```cpp
void callback(boost::shared_ptr<std_msgs::String const>);
void callback(std_msgs::StringConstPtr);
void callback(std_msgs::String::ConstPtr);
void callback(const std_msgs::String&);
void callback(std_msgs::String);
void callback(const ros::MessageEvent<std_msgs::String const>&);
```
在一个node中多重订阅一个topic时，可以使用非const消息，可以使用值传递，拷贝
```cpp
void callback(const boost::shared_ptr<std_msgs::String>&);
void callback(boost::shared_ptr<std_msgs::String>);
void callback(const std_msgs::StringPtr&);
void callback(const std_msgs::String::Ptr&);
void callback(std_msgs::StringPtr);
void callback(std_msgs::String::Ptr);
void callback(const ros::MessageEvent<std_msgs::String>&);
```

12. [callback types](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)
    roscpp supports any callback supported by boost::function:
  + functions
  + class methods
  + functor objects (including boost::bind)仿函数对象
      A functor object is a class that declares operator()
  ```cpp
   class Foo
   {
   public:
     void operator()(const std_msgs::StringConstPtr& message)
     {
     }
   };
  ```
  A functor passed to subscribe() must be copyable. The Foo functor could be used with subscribe() like so:
  ```cpp
ros::Subscriber sub = nh.subscribe<std_msgs::String>("my_topic", 1, Foo());
  ```
Note: when using functor objects (like boost::bind, for example) you must explicitly specify the message type as a template argument, because the compiler cannot deduce it in this case.

13. [rosbag namespace](http://docs.ros.org/diamondback/api/rosbag/html/c++/namespacerosbag.html#a9054d4b91cf6fa6f35afc3b9b16d0280)
    [rosbag/Cookbook - ROS Wiki](http://wiki.ros.org/rosbag/Cookbook)
```cpp
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
rosbag::Bag bag;
bag.open(filename, rosbag::bagmode::Read);

std::vector<std::string> topics; // 获取对应的topic消息
rosbag::View view(bag, rosbag::TopicQuery(topics));
BOOST_FOREACH(rosbag::MessageInstance const m, view)
{
  if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
  {
    sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
    if (l_img != NULL)
    {
      l_img_sub.newMessage(l_img);
    }
  }
}

#include <rosbag/structures.h>
#include <boost/foreach.hpp>
获取bag中的相应的topic
rosbag::Bag bag;
rosbag::View view(bag);
bag.open(filename, rosbag::bagmode::Read);
std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) 
{
  if (info->datatype == "sensor_msgs/PointCloud2" || info->datatype == "sensor_msgs/CompressedImage")
  {
    topics.emplace_back(info->topic);
  }
}
```

+ [image_transport](http://wiki.ros.org/image_transport)
  利用此包处理图像消息，
```cpp
  <depend>image_transport</depend>
   // Use the image_transport classes instead.
   #include <ros/ros.h>
   #include <image_transport/image_transport.h>
   
   void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     // ...
   }
   
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);
   image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);
```


+ [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure/Tutorials)
  动态配置参数
```xml
<depend>dynamic_reconfigure</depend>
```

+ [诊断硬件](http://wiki.ros.org/diagnostic_updater)
  diagnostic_updater contains tools for easily updating diagnostics. it is commonly used in device drivers to keep track of the status of output topics, device status, etc.
  参考pointgrey相机驱动
  [diagnostic_updater: diagnostic_updater::Updater Class Reference](http://docs.ros.org/latest/api/diagnostic_updater/html/classdiagnostic__updater_1_1Updater.html)
```xml
  <depend>diagnostic_updater</depend>
```

+ [相机图像处理包image_proc](http://wiki.ros.org/image_proc)
  [GitHub - ros-perception/image_pipeline](https://github.com/ros-perception/image_pipeline)
  [image_pipeline/image_proc/src at indigo · ros-perception/image_pipeline · GitHub](https://github.com/ros-perception/image_pipeline/tree/indigo/image_proc/src)


## plugin

+ [PlotJuggler](https://github.com/facontidavide/PlotJuggler)
```sh
sudo apt-get install ros-kinetic-plotjuggler 
rosrun plotjuggler PlotJuggler # To run the application, use the command:
```