# Encapsulate SDK Rules

## Notation

### Head File
+ Copyright
```
/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2017, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 1.0
 * Date: 2017.12
 *
 * DESCRIPTION
 *
 * Robosense gound remove module.
 *
 */
```


 + Public function   
  Explain function and parameters using doxygen.



## Code Style

### C++
+ Use `std::size_t` to replace `unsigned int` parameters, especially for the loop step, `for(std::size_t i = 0; i < N; ++i)`.
+ Print exception as detail as possible
  - Check parameters of function. Print cues if illegal.
  - Check return value of function, especially the value as intermediate result.
  - Use `PCL_ERROR` to replace `std::cerr`, as it print red text in terminal.
+ [Functions arguments](https://google.github.io/styleguide/cppguide.html#Function_Parameter_Ordering)   
  Input arguments use values or const references while output arguments use pointers. All parameters passed by reference must be labeled `const`.
+ Check Opencv version, especially there are different function interfaces.
```cpp
int main( int argc, char** argv )
{
  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : " << CV_MAJOR_VERSION << endl;
  cout << "Minor version : " << CV_MINOR_VERSION << endl;
  cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;
  if ( CV_MAJOR_VERSION < 3)
  {
      // Old OpenCV 2 code goes here.
  }
  else
  {
      // New OpenCV 3 code goes here.
  }
}
OpenCV version : 2.4.8
Major version : 2
Minor version : 4
Subminor version : 8
```

+ Note   
  Add `output="screen"` to node in launch file, so that the `std::cout` can print to screen, otherwise it print to log file.
```
 <node pkg="drivable_area" name="drivable_area_test_node" type="drivable_area_test_node" output="screen" />
```



### Doxygen
Doxygen is the de facto standard tool for generating documentation from annotated C++ sources.   
#### Install
`sudo apt-get install doxygen`

#### Syntax
+ brief   
  Use sentence or phrase to describe public functions or parameters. Capitalize the first letter of the sentence or phrase without punctuation at the end.

+ param   
  Use sentence or phrase to describe parameters of public function. First letter of the sentence or phrase should be lowercase without punctuation at the end.
  - in: input parameters
  - out: output parameters
+ return   
  The same as param.

Remember to modify doxygen annotation when you amend function interface.

#### Generate doxygen documentation
+ Create configuration file   
  `doxygen -g <config-file>`   
  where <config-file> is the name of the configuration file. If you omit the file name, a file named Doxyfile will be created.   
+ Modify configuration file `Doxyfile`   
  - Input and output path
    Appoint input and output path, especially when the Doxyfile is not saved at `./` or you don't want to save doxygen documentation at the default path `./doc`.
  ```
  OUTPUT_DIRECTORY       = ./doc/doxygen
  INPUT                  = ./include
  ```
  - Documentation format
    Make sure which format (html, latex, rtf, man, docbook) you want to create, then set the flag to be YES, otherwise NO or delete related codes.
  ```
  GENERATE_HTML          = YES
  ```
  - Project brief
    Using the PROJECT_BRIEF tag one can provide an optional one line description for a project that appears at the top of each page and should give viewer a quick idea about the purpose of the project.
  ```
  PROJECT_BRIEF          = "This is a doc for robosense drivable area sdk APIs. Note that this is a early internal testing \
  			 version, maintained by Robosense LTD. And it will be improved gradually with quick version updates."
  ```

+ Run doxygen
  To generate the documentation you can enter:
```
doxygen Doxygen
```

## Lib
+ Add [install targets](http://wiki.ros.org/catkin/CMakeLists.txt) to CMakeLists.txt   
```
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
+ Catkin configure
  - Generate `install` folder
  ```
  catkin config --install
  ```

  - If package can be found, but can't find launch file, though you have sourced devel/setup.zsh
  ```
  catkin config --merge-devel
  ```

## [Deb](https://gist.github.com/awesomebytes/196eab972a94dd8fcdd69adfe3bd1152)
#### CMakeLists
Install related files so that the package can be launched normally after installed.

```cmake
install(DIRECTORY data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
        )
install(DIRECTORY doc/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
install(DIRECTORY node/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node
        )
install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        )
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(TARGETS drivable_area_test_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	      )
```

### Dependencies
```
sudo apt-get install python-bloom
sudo apt-get install fakeroot
```

### Generate doxygen documentation
Generate doxygen documentation before encapsulating debian file so it can be included in debian.

### Create debian structure
+ Indigo (Ubuntu 14.04)
```
cd package folder
bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
```
+ Kinetic (Ubuntu 16.04)
```
bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
```
If you encounter the error:
```
debian/rules:50: recipe for target 'override_dh_shlibdeps' failed
make[1]: *** [override_dh_shlibdeps] Error 2
make[1]: Leaving directory
debian/rules:22: recipe for target 'binary' failed
make: *** [binary] Error 2
```
[Add bellow code to debian/rules](https://github.com/SpiderLabs/ModSecurity-nginx/issues/16) to modify `override_dh_shlibdeps` setting.
```
override_dh_shlibdeps:
    dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info
```
Or use following code to replace raw piece of code:
```
override_dh_shlibdeps:
	# In case we're installing to a non-standard location, look for a setup.sh
	# in the install tree that was dropped by catkin, and source it.  It will
	# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
	if [ -f "/opt/ros/kinetic/setup.sh" ]; then . "/opt/ros/kinetic/setup.sh"; fi && \
	dh_shlibdeps -l$(CURDIR)/debian/ros-kinetic-drivable-area//opt/ros/kinetic/lib/ \
	dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info
```

### Create binary debian
```
fakeroot debian/rules binary
```