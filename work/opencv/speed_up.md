# Speed Up

## [代码级别加速](https://blog.csdn.net/guyuealian/article/details/78540206)
### opencv 3.0版本以上使用UMat替代Mat
OpenCV3中引入了一个新的图像容器对象UMat，它跟Mat有着多数相似的功能和相同的API函数，但是代表的意义却太不一样。要说到UMat对象的来龙去脉，必须首先从OpenCL来开始说，OpenCL是一个面向异构系统通用的并行编程标准，这个标准最早是苹果公司提出，后来变成了一个国际标准，目的是通过它开发通用的GPU计算软件，中国的华为是该标准的成员之一。说的直白点就是如果CPU或者GPU支持OpenCL标准，就可以通过OpenCL相关编程实现使用GPU计算。
+ UMat转换为Mat
  UMat to Mat: UMat::getMat(int access_flags)，支持的flag如下：
  ACCESS_READ
  ACCESS_WRITE
  ACCESS_RW
  ACCESS_MASK
  ACCESS_FAST 
  种方式的时候UMat对象将会被LOCK直到CPU使用获取Mat对象完成操作，销毁临时Mat对象之后，UMat才可以再被使用。
+ Mat转换为UMat
  通过Mat::getUMat()之后就获取一个UMat对象，同样在UMat对象操作期间，作为父对象Mat也会被LOCK直到子对象UMat销毁之后才可以继续使用。

### 查表法LUT
使用lut的方法法，远快于每个像素都计算的方法
```cpp
	void gammaCorrection(cv::Mat& dst, float fGamma)
	{
		cv::Mat lut(1, 256, CV_8U);
		uchar *p = lut.data;
		for (int i = 0; i < 256; i++)
		{
			p[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
		}
		cv::LUT(dst, lut, dst);
	}
```
### OpenMP

### 常用小技巧
[OpenCV学习笔记（三） 访问像素 - eric.xing - 博客园](https://www.cnblogs.com/ericxing/p/3576747.html)
+ 释放Mat图像内存
```cpp
cv::Mat image = cv::imread("test.jpg");
image.release();
```
+  Mat多通道合并与分割
```cpp
/*
	将多个mat合并为多通道mat
*/
cv::Mat mergeMultiChannels(cv::Mat A,cv::Mat B) {
	cv::Mat AB;
	vector<cv::Mat> ABchannels;
	ABchannels.push_back(A);
	ABchannels.push_back(B);
	cv::merge(ABchannels, AB);
	return AB;
}
 
/*
	将6通道的mat分割成2个三通道的mat
*/
void splitMultiChannels(cv::Mat mat,cv::Mat &A,cv::Mat &B) {
	vector<cv::Mat> channels;
	cv::split(mat, channels);//分割image1的通
	vector<cv::Mat> Avec, Bvec;
	Avec.push_back(channels[0]);
	Avec.push_back(channels[1]);
	Avec.push_back(channels[2]);
 
	Bvec.push_back(channels[3]);
	Bvec.push_back(channels[4]);
	Bvec.push_back(channels[5]);
 
	cv::merge(Avec, A);
	cv::merge(Bvec, B);
}
```

+ 释放vector
```cpp
//放在头文件
template < class T >
void ClearVector(vector< T >& vt)
{
	vector<T> vtTemp;
	vtTemp.swap(vt);
}


vector<void *> v;
// 每次new之后调用v.push_back()该指针，在程序退出或者根据需要，用以下代码进行内存的释放：
for (vector<void *>::iterator it = v.begin(); it != v.end(); it ++) 
    if (NULL != *it) 
    {
        delete *it; 
        *it = NULL;
    }
v.clear();
```
swap()是交换函数，使vector离开其自身的作用域，从而强制释放vector所占的内存空间，总而言之，释放vector内存最简单的方法是vector<Point>().swap(pointVec)

### [使用opencv实现3D卷积，并且优化](https://mp.weixin.qq.com/s?__biz=MzIxOTcyNDE2NA==&mid=2247483914&idx=1&sn=68fceefa37ec27a3506b61acffa041a8&chksm=97d7a2e1a0a02bf7e37561050d970bdc95434581480a4f3ab113316872305458d220d298bae4&mpshare=1&scene=1&srcid=&pass_ticket=X16K6FYgsAd1RTIEGQs030Z31xHk7rJhtSxNOivtCTqGaaxO8c8%2BE0U3ubh0R8BP#rd)
![417bc319](images/417bc319.png)

+ 使用6重循环实现,.at<float>访问元素
```cpp
void vonv3x3_naive(const cv::Mat& inp, const cv::Mat& w, cv::Mat& out)
{
  int kh = w.size[2], kw = w.size[3];
  int inp_cn = inp.size[0], inp_h = inp.size[1], inp_w = inp.size[2];
  int out_cn = w.size[0], out_h = inp_h - kh + 1, out_w = inp_w - kw + 1;
  int size[] = { out_cn, out_h, out_w };
  out.create(3, size, CV_32F);
  for (int oc = 0; oc < out_cn; ++oc)
  {
    for (int y = 0; y < out_h; ++y)
    {
      for (int x = 0; x < out_w; ++x)
      {
        out.at<float>(oc, y, x) = 0;
        for (int ic = 0; ic < inp_cn; ++ic)
        {
          for (int dy = 0; dy < kh; ++dy)
          {
            for (int dx = 0; dx < kw; ++dx)
            {
              int widx[] = { oc, ic, dy, dx };  // prepare 4D index to access w
              out.at<float>(oc, y, x) += inp.at<float>(ic, y + dy, x + dx) * w.at<float>(widx);
            }
          }
        }
      }
    }
  }
}
```