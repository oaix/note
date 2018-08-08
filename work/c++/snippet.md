## Useful snippets

### Transform number to string
```cpp
template<typename T>
std::string num2str(T num, int precision)
{
	std::stringstream ss;
	ss.setf(std::ios::fixed, std::ios::floatfield);
	ss.precision(precision);
	std::string st;
	ss << num;
	ss >> st;
	return st;
}
```

### Sleep for awhile
```cpp
#include <chrono>
#include <thread>

int main()
{
  // c++11
  using namespace std::this_thread; // sleep_for, sleep_until
  using namespace std::chrono; // nanoseconds, system_clock, seconds

  sleep_for(nanoseconds(10));
  sleep_until(system_clock::now() + seconds(1));
}
```
```cpp
#include <ctime>
void wait(int seconds)
{
  clock_t endwait;
  endwait = clock () + seconds * CLOCKS_PER_SEC ;
  while (clock() < endwait) {}
}
```
### 判断一个string是不是纯数字组成
```cpp
bool is_number(const std::string& s)
{
  std::string::const_iterator it = s.begin();
  while (it != s.end() && std::isdigit(*it)) ++it;
  return !s.empty() && it == s.end();
}
```

### Read txt line by line
[online](https://stackoverflow.com/questions/9139300/stringstream-to-vectorint?lq=1)   
[split a string into a vector](https://codereview.stackexchange.com/questions/159628/c-split-string-into-a-vector)    
Elements are separated by blank space.
```cpp
#include<sstream>

// split a string into a vector
template<typename T>
std::vector<T> Split(const std::string &subject)
{
  std::istringstream ss{subject};
  using StrIt = std::istream_iterator<T>;
  std::vector<T> container{StrIt{ss}, StrIt{}};
  return container;
}

template<typename T>   
bool readTxtLineByLine(const std::string txt_path, std::vector<T> &out_v)
{
  std::ifstream in_file;
  in_file.open(txt_path, std::ifstream::in);
  if (!in_file.is_open())
  {
    PCL_ERROR("Open label text file unsuccessfully!\n");
    return false;
  }
  std::string line;
  while (getline(in_file, line))
  {
      std::istringstream ss(line);
      T tmp;
      while (ss >> tmp)
      {
          out_v.emplace_back(tmp);
      }

  }
  in_file.close();
  return true;
}
```

### 将姿态角和平移向量转换为转换矩阵  
其中pose[0] ~ pose[2]: x, y, z轴上的平移量； pose[3] ~ pose[5]: roll, pitch, yam
```cpp
void pose2TransformMatrix(const std::vector<float> &pose, Eigen::Matrix4f &transform_matrix)
{
  Eigen::AngleAxisf current_rotation_x(pose[3], Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf current_rotation_y(pose[4], Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf current_rotation_z(pose[5], Eigen::Vector3f::UnitZ());
  Eigen::Translation3f current_translation(pose[0], pose[1], pose[2]);

  transform_matrix = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
}

void pose2TransformMatrix(const Eigen::Matrix<float, 6, 1> &pose, Eigen::Matrix4f &transform_matrix)
{
  Eigen::AngleAxisf current_rotation_x(pose[3], Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf current_rotation_y(pose[4], Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf current_rotation_z(pose[5], Eigen::Vector3f::UnitZ());
  Eigen::Translation3f current_translation(pose[0], pose[1], pose[2]);

  transform_matrix = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
}
```
### [计算两3D直线交点](https://math.stackexchange.com/questions/270767/find-intersection-of-two-3d-lines)
```cpp
void getIntersection(const cv::Vec6f &line1, const cv::Vec6f &line2, cv::Point3f &point)
{
  cv::Point3f v1(line1[0], line1[1], line1[2]);
  cv::Point3f p1(line1[3], line1[4], line1[5]);
  cv::Point3f v2(line2[0], line2[1], line2[2]);
  cv::Point3f p2(line2[3], line2[4], line2[5]);

  cv::Point3f vector_temp1 = (p1 - p2).cross(v2);
  cv::Point3f vector_temp2 = v2.cross(v1);
  double t1 = 0.0;
  if (std::abs(vector_temp2.x) > FLT_EPSILON)
  {
    t1 = vector_temp1.x / vector_temp2.x;
  }
  else if (std::abs(vector_temp2.y) > FLT_EPSILON)
  {
    t1 = vector_temp1.y / vector_temp2.y;
  }
  else if (std::abs(vector_temp2.z) > FLT_EPSILON)
  {
    t1 = vector_temp1.z / vector_temp2.z;
  }
  else
  {
    std::cout << " Error!" << std::endl;
  }
  point = p1 + t1 * v1;
}
// 也可以使用参数方程表示直线，然后求解
```

### [C++: Check if file exists](https://techoverflow.net/2013/01/11/c-check-if-file-exists/)
In C++ you want to check if a given file exists, but you can’t use stat() because your code needs to work cross-plaform. This solution is 100% portable (stat() isn’t, even if it it’s widely support), but note that it opens the file, so it might fail if it exists, but the user who is running the program isn’t allowed to access it.

```cpp
#include <fstream>
bool fexists(const char *filename)
{
  std::ifstream ifile(filename);
  return (bool)ifile;
}
```

If you have the filename as `std::string` rather than as cstring, you can use this snippet:
```cpp
#include <fstream>
bool fexists(const std::string &filename)
{
  std::ifstream ifile(filename.c_str());
  return (bool)ifile;
}
```

If you know for a fact that you have access to [stat()](http://pubs.opengroup.org/onlinepubs/009695399/functions/stat.html) I recommend using stat instead. See [this followup blog post](https://techoverflow.net/2013/08/21/how-to-check-if-file-exists-using-stat/?q=/blog/2013/08/21/how-to-check-if-file-exists-using-stat/) for an example on how to do this.

```cpp
#include <sys/stat.h>
/**
 * Check if a file exists
 * @return true if and only if the file exists, false else
 */
bool fileExists(const char* file)
{
  struct stat buf;
  return (stat(file, &buf) == 0);
}
```

If you want to use C++ `std::string` for the filename, you can use this equivalent instead:
```cpp
#include <sys/stat.h>
/**
 * Check if a file exists
 * @return true if and only if the file exists, false else
 */
bool fileExists(const std::string &file)
{
  struct stat buf;
  return (stat(file.c_str(), &buf) == 0);
}
```
+ [performance comparation](https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c)
```cpp
#include <sys/stat.h>
#include <unistd.h>
#include <string>
inline bool exists_test0(const std::string& name)
{
  ifstream f(name.c_str());
  return f.good();
}
inline bool exists_test1(const std::string& name)
{
  if (FILE *file = fopen(name.c_str(), "r"))
	{
    fclose(file);
    return true;
  }
	else
	{
    return false;
  }   
}
inline bool exists_test2(const std::string& name)
{
  return ( access( name.c_str(), F_OK ) != -1 );
}
inline bool exists_test3(const std::string& name)
{
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0);
}
```
Results for total time to run the 100,000 calls averaged over 5 runs:
```
Method exists_test0 (ifstream): **0.485s**
Method exists_test1 (FILE fopen): **0.302s**
Method exists_test2 (posix access()): **0.202s**
Method exists_test3 (posix stat()): **0.134s**
```
![](images/if_file_exist_compare.png)
	- The `stat()` function provided the best performance on my system (Linux, compiled with g++), with a standard `fopen` call being your best bet if you for some reason refuse to use POSIX functions.
	- The performance of [boost::filesystem::exists](http://www.cplusplus.com/forum/windows/194885/) function is very close to that of stat function and it is also portable. I would recommend this solution if boost libraries is accessible from your code.
```cpp
#include <boost/filesystem.hpp>
boost::filesystem::path p(fname);
boost::filesystem::exists(p)
```
