# snippet

### Check Opencv version
```cpp
#include <opencv2/core/version.hpp>

using namespace cv;
using namespace std;

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

#if CV_MAJOR_VERSION == 2
// do opencv 2 code
#elif CV_MAJOR_VERSION == 3
// do opencv 3 code
#endif

// if(OpenCV_VERSION VERSION_LESS "3.0")
// # use 2.4 modules
// else()
// # use 3.x modules
// endif()

OpenCV version : 2.4.8
Major version : 2
Minor version : 4
Subminor version : 8

std::cout << CV_VERSION << std::endl; // 2.4.8
std::cout << CV_MAJOR_VERSION << std::endl; // 2
```

### [曲线拟合](https://blog.csdn.net/i_chaoren/article/details/79822574)
```cpp
void polyfit(std::vector<cv::Point2f> &in_point, int n, cv::Mat &mat_k)
{
  int size = in_point.size();
  // number of coefficients
  int x_num = n + 1;
  //matrix U and Y
  cv::Mat mat_u(size, x_num, CV_32F);
  cv::Mat mat_y(size, 1, CV_32F);

  for (int i = 0; i < mat_u.rows; ++i)
    for (int j = 0; j < mat_u.cols; ++j)
    {
      mat_u.at<float>(i, j) = pow(in_point[i].x, j);
    }
  for (int i = 0; i < mat_y.rows; ++i)
  {
    mat_y.at<float>(i, 0) = in_point[i].y;
  }
  //get coefficients
  mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
}
```

### rgb图像归一化
```cpp
void equalizeHistRGB(cv::Mat &image_src, cv::Mat &image_dst)
{
  cv::Mat img_hsv;
  cv::cvtColor(image_src, img_hsv, CV_BGR2HSV);
  const uint8_t channel = img_hsv.channels();
  cv::Mat hsv_ch[channel];
  cv::split(img_hsv, hsv_ch); // split
  cv::equalizeHist(hsv_ch[2], hsv_ch[2]); // channel of value
  cv::merge(hsv_ch, channel, img_hsv); // merge
  cv::cvtColor(img_hsv, image_dst, CV_HSV2BGR);
}
```

