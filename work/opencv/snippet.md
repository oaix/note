## Useful snippets

### Check Opencv version
```cpp
#include "opencv2/opencv.hpp"

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

OpenCV version : 2.4.8
Major version : 2
Minor version : 4
Subminor version : 8

std::cout << CV_VERSION << std::endl; // 2.4.8
std::cout << CV_MAJOR_VERSION << std::endl; // 2
```
