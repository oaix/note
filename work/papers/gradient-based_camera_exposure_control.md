[TOC]

# 相机自动曝光

## Gradient-based Camera Exposure Control for Outdoor Mobile Platforms

Inwook Shim, 2018

## Gradient-based Camera Exposure Control for Outdoor Mobile Platforms

2017

```cpp
class polyResidual
{
public:
  polyResidual(double x, double y) : x_(x), y_(y)
  {
  }
  template <typename T>
  bool operator()(const T* const coff, T* residual) const
  {
    residual[0] = y_ - poly(coff, 5);
    return true;
  }

private:
  template <typename T>
  T poly(const T* const coff, const int N) const
  {
    T p = coff[N];
    for (int j = N - 1; j >= 0; --j)
    {
      p = p * x_ + coff[j];
    }
    return p;
  }
  const double x_, y_;
};

double FOURTH_COFF[5] = { 0 };
class fourthFunctorDeriv
{
public:
  fourthFunctorDeriv(float const& to_find_root_of) : a_(to_find_root_of)
  {
  }
  std::pair<float, float> operator()(const float& x)
  {
    // rerturn both f(x) and f'(x)
    float ff[2];
    dpoly(x, ff);
    return std::make_pair(ff[0] - a_, ff[1]);
  }

private:
  void dpoly(const float x, float* dp)
  {
    dp[0] = FOURTH_COFF[4];
    dp[1] = 0;
    for (int j = 4 - 1; j >= 0; --j)
    {
      dp[1] = dp[1] * x + dp[0];
      dp[0] = dp[0] * x + FOURTH_COFF[j];
    }
  }
  float a_;
};

double PowerAbs(double base, unsigned int absExp)
{
  // return condition
  if (absExp == 0)
    return 1.0;
  if (absExp == 1)
    return base;

  // recursion
  double result = PowerAbs(base, absExp >> 1);
  result *= result;
  // odd or even
  if ((absExp & 1) == 1)
  {
    result *= base;
  }
  return result;
}

double Power(double base, int exp)
{
  // 底数为0，指数为负数的情况
  if (std::abs(base - 0.0) < FLT_EPSILON && exp <= 0)
  {
    return 0.0;
  }

  unsigned int absExp = static_cast<unsigned int>(std::abs(exp));
  double result = PowerAbs(base, absExp);
  if (exp < 0)
  {
    result = 1.0 / result;
  }
  return result;
}

void gaussEliminationLS(std::vector<std::vector<double> >& a, double* x)
{
  int m = a.size();
  int n = a[0].size();
  int i, j, k;
  for (i = 0; i < m - 1; ++i)
  {
    // Partial Pivoting
    for (k = i + 1; k < m; ++k)
    {
      // If diagonal element(absolute vallue) is smaller than any of the terms below it
      if (fabs(a[i][i]) < fabs(a[k][i]))
      {
        // Swap the rows
        for (j = 0; j < n; ++j)
        {
          double temp;
          temp = a[i][j];
          a[i][j] = a[k][j];
          a[k][j] = temp;
        }
      }
    }
    // Begin Gauss Elimination
    for (k = i + 1; k < m; ++k)
    {
      double term = a[k][i] / a[i][i];
      for (j = 0; j < n; ++j)
      {
        a[k][j] = a[k][j] - term * a[i][j];
      }
    }
  }
  // Begin Back-substitution
  for (i = m - 1; i >= 0; --i)
  {
    x[i] = a[i][n - 1];
    for (j = i + 1; j < n - 1; ++j)
    {
      x[i] = x[i] - a[i][j] * x[j];
    }
    x[i] = x[i] / a[i][i];
  }
}

void fitGammaCurve(const std::vector<cv::Point2d>& grad_gamma_v, const int n, double coff[])
{
  std::size_t point_num = grad_gamma_v.size();
  double X[2 * n + 1] = { 0 };  // Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
  for (std::size_t i = 0; i < 2 * n + 1; ++i)
  {
    for (std::size_t j = 0; j < point_num; ++j)
    {
      X[i] += Power(grad_gamma_v[j].x, i);
    }
  }
  // B is the Normal matrix(augmented) that will store the equations
  std::vector<std::vector<double> > B(n + 1, std::vector<double>(n + 2, 0));
  for (std::size_t i = 0; i <= n; ++i)
  {
    for (std::size_t j = 0; j <= n; ++j)
    {
      B[i][j] = X[i + j];
    }
  }
  double Y[n + 1] = { 0 };
  for (std::size_t i = 0; i <= n; ++i)
  {
    for (std::size_t j = 0; j < point_num; ++j)
    {
      Y[i] += Power(grad_gamma_v[j].x, i) * grad_gamma_v[j].y;
    }
  }
  for (std::size_t i = 0; i <= n; ++i)
  {
    B[i][n + 1] = Y[i];
  }
  gaussEliminationLS(B, coff);
}

// gamma correction, {1/1.9, 1/1.5, 1/1.2, 1.2, 1.5, 1.9, 1}
  unsigned char lut_[7][256];
  std::vector<double> gamma_;
gamma_.assign(std::initializer_list<double>({ 1 / 1.9, 1 / 1.5, 1 / 1.2, 1.2, 1.5, 1.9, 1 }));  // 7 gamma values
    // init gamma correction table
    for (std::size_t i = 0; i < gamma_.size(); ++i)
    {
      for (std::size_t j = 0; j < 256; ++j)
      {
        lut_[i][j] = cv::saturate_cast<uchar>(std::pow((float)(j / 255.0), gamma_[i]) * 255.0f);
      }
    }

void gammaCorrection(const cv::Mat& src, cv::Mat& dst, const int gamma_order)
  {
    dst = src.clone();
    int rows = dst.rows;
    int cols = dst.cols;
    if (dst.isContinuous())
    {
      cols *= rows;
      rows = 1;
    }
    unsigned char* lut = lut_[gamma_order];
    const int channels = dst.channels();
    switch (channels)
    {
      case 1:
      {
        for (int i = 0; i < rows; ++i)
        {
          uchar* data = dst.data + i * cols;
          for (int j = 0; j < cols; ++j, ++data)
          {
            *data = lut[*data];
          }
        }
        break;
      }
      case 3:
      {
        for (int i = 0; i < rows; ++i)
        {
          cv::Vec3b* data = dst.ptr<cv::Vec3b>(i);
          for (int j = 0; j < cols; ++j, ++data)
          {
            (*data)[0] = lut[(*data)[0]];
            (*data)[1] = lut[(*data)[1]];
            (*data)[2] = lut[(*data)[3]];
          }
        }
        break;
      }
    }
  }

  float getAmountOfGradientInfor(cv::Mat& image)
  {
    // cv::GaussianBlur(image, image, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, grad;
    cv::Sobel(image, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::Sobel(image, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    // Scales, calculates absolute values, and converts the result to 8-bit
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    // 𝚍𝚜𝚝(I)=𝚜𝚊𝚝𝚞𝚛𝚊𝚝𝚎(𝚜𝚛𝚌𝟷(I)∗𝚊𝚕𝚙𝚑𝚊+𝚜𝚛𝚌𝟸(I)∗𝚋𝚎𝚝𝚊+𝚐𝚊𝚖𝚖𝚊)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);  // gard is 8 bits

    // delta regards a gradient value smaller than delta as noise and ignores it
    float gradient_threshold = 0.06;  // delta
    // a small lambda emphasizes strong intensity variations, emphasize subtle texture variations by setting lambda to a
    // large value
    float lambda = 1000;
    // N = log(lambda * (1 - delta) + 1)
    float normal_rate = 1.0f / std::log(lambda * (1 - gradient_threshold) + 1);
    float mag_sum = 0;
    int rows = grad.rows;
    int cols = grad.cols;
    if (grad.isContinuous())
    {
      cols *= rows;
      rows = 1;
    }
    // #pragma acc kernels
    for (int i = 0; i < rows; ++i)
    {
      uchar* data = grad.data + i * cols;
      for (int j = 0; j < cols; ++j, ++data)
      {
        if (*data > gradient_threshold)
        {
          mag_sum += std::log(lambda * (*data - gradient_threshold) + 1) * normal_rate;
        }
      }
    }
    return mag_sum;
  }

  void polyfit(std::vector<cv::Point2f>& in_point, int n, cv::Mat& mat_k)
  {
    int size = in_point.size();
    // number of coefficients
    int x_num = n + 1;
    // matrix U and Y
    cv::Mat mat_u(size, x_num, CV_32F);
    cv::Mat mat_y(size, 1, CV_32F);

    for (int i = 0; i < mat_u.rows; ++i)
    {
      for (int j = 0; j < mat_u.cols; ++j)
      {
        mat_u.at<float>(i, j) = pow(in_point[i].x, j);
      }
    }

    for (int i = 0; i < mat_y.rows; ++i)
    {
      mat_y.at<float>(i, 0) = in_point[i].y;
    }

    // get coefficients
    //    cv::Mat mat_k(x_num, 1, CV_32F);
    mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
  }

  /**
   * @brief poly 计算多项式的值，采用 horner 算法
   * @param coff 多项式的系数，c0, c1, ... cN
   * @param n 多项式最高项的次数
   * @param x 自变量的取值
   * @return 多项式在 x 处的值
   */
  template <typename T>
  T poly(const T coff[], const int N, const T x)
  {
    T p = coff[N];
    for (int j = N - 1; j >= 0; --j)
    {
      p = p * x + coff[j];
    }
    return p;
  }

  /**
   * @brief dpoly 计算多项式的值和一阶导数值
   * @param coff  多项式的系数，c0, c1, ... cN
   * @param N 多项式最高项的次数
   * @param x 自变量的取值
   * @param [out] d 多项式在 x 处的导数值
   * @param dp 多项式在 x 处的各阶导数值，从 0 阶到 1 阶, dp[2]
   */
  template <typename T>
  void dpoly(const T coff[], const int N, const T x, T dp[])
  {
    dp[0] = coff[N];
    dp[1] = 0;
    for (int j = N - 1; j >= 0; --j)
    {
      dp[1] = dp[1] * x + dp[0];
      dp[0] = dp[0] * x + coff[j];
    }
  }

  /**
   * @brief ddpoly 计算多项式的第 0 阶到第 M 阶导数
   * @param coff 多项式的系数，c0, c1, ... cN
   * @param N 多项式最高项的次数
   * @param x 自变量的取值
   * @param dp 多项式在 x 处的各阶导数值，从 0 阶到 M 阶
   * @param M 需要求解的多项式的导数的最高阶次
   */
  template <typename T>
  void ddpoly(const T coff[], const int N, const T x, const int M, T dp[])
  {
    dp[0] = coff[N];
    for (int j = 1; j < M; ++j)
    {
      dp[j] = 0.0;
    }
    int nnd = 0;
    for (int i = N - 1; i >= 0; --i)
    {
      nnd = (M < N - i) ? M : N - i;
      for (int j = nnd; j > 0; --j)
      {
        dp[j] = dp[j] * x + dp[j - 1];
      }
      dp[0] = dp[0] * x + coff[i];
    }
    T cnst = 1.0;
    for (int i = 2; i < M + 1; ++i)
    {
      cnst = cnst * i;
      dp[i] = dp[i] * cnst;
    }
  }

  float updateExposureRatio(const float gamma_opt)
  {
    // kp controls the speed of convergence, a lager kp causes convergence faster
    float kp = 0.2f;
    // d controls the nonlinearity of the update function, a smaller d value results in a smoother transition
    float d = 0.5f;
    float R = d * std::tan((1 - gamma_opt) * std::atan(1 / d));
    float alpha = gamma_opt < 1.0f ? 1.0f : 0.5f;
    return (1 + alpha * kp * R);
  }

  void updateGainShutter(const float exposure_ratio)
  {
    if (exposure_ratio > 1.0f)
    {
      if (std::abs(shutter_ - shutter_speed_max_) < FLT_EPSILON)
      {
        if (gain_ > 0.9)
        {
          gain_ = exposure_ratio * gain_ - gain_max_ > 0 ? gain_max_ : exposure_ratio * gain_;
        }
        else
        {
          gain_ = 1;
        }
        pg_.setGain(gain_);
      }
      else
      {
        shutter_ = exposure_ratio * shutter_ - shutter_speed_max_ > 0 ? shutter_speed_max_ : exposure_ratio * shutter_;
        pg_.setShutter(shutter_);
      }
    }
    else
    {
      if (gain_ > 1)
      {
        gain_ = exposure_ratio * gain_;
        pg_.setGain(gain_);
      }
      else if (gain_ < FLT_EPSILON)
      {
        shutter_ = exposure_ratio * shutter_;
        pg_.setShutter(shutter_);
      }
      else
      {
        gain_ = 0;
        pg_.setGain(gain_);
      }
    }
  }

  void setGainShutter()
  {
#if DEBUG
    ros::Time t1 = ros::Time::now();
#endif
    cv::Mat image_raw = (cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::TYPE_8UC3)->image)(
        cv::Rect(image_ptr_->width >> 3, image_ptr_->height >> 3, image_ptr_->width - (image_ptr_->width >> 2),
                 image_ptr_->height - (image_ptr_->height >> 2)));
    cv::Mat image_gray;
    cv::cvtColor(image_raw, image_gray, CV_BGR2GRAY);
    cv::Mat image;
    cv::resize(image_gray, image, cv::Size(0, 0), 0.5, 0.5, CV_INTER_AREA);

#if DEBUG
    ros::Time t2 = ros::Time::now();
    std::cout << "down example: " << (t2 - t1).toSec() * 1000 << " ms" << std::endl;
#endif
    std::vector<cv::Point2d> grad_gamma_v(gamma_.size(), cv::Point2d(0, 0));
#ifdef _OPENMP
#pragma omp parallel for num_threads(gamma_.size())
#endif
    for (std::size_t i = 0; i < gamma_.size(); ++i)  // last gamma = 1, there is no need to gamma correction
    {
      grad_gamma_v[i].x = gamma_[i];
      if (i < gamma_.size() - 1)
      {
        cv::Mat image_gamma;
        gammaCorrection(image, image_gamma, i);
        grad_gamma_v[i].y = getAmountOfGradientInfor(image_gamma);
      }
      else
      {
        grad_gamma_v[i].y = getAmountOfGradientInfor(image);  // raw image gamma = 1
      }
    }

#if DEBUG
    ros::Time t3 = ros::Time::now();
    std::cout << "get amount of gradient: " << (t3 - t2).toSec() * 1000 << " ms" << std::endl;
#endif

    // fit curve of gradient magnitude and gamma, fifth-order polynomial fitting
    const int fit_order = 5;
    double coff[fit_order + 1] = { 0 };
    fitGammaCurve(grad_gamma_v, fit_order, coff);

#if DEBUG
    ros::Time t4 = ros::Time::now();
    std::cout << "fit gamma curve: " << (t4 - t3).toSec() * 1000 << " ms" << std::endl;
#endif
    // get optimization value of gamma
    for (std::size_t i = 0; i < fit_order; ++i)
    {
      FOURTH_COFF[i] = (i + 1) * coff[i + 1];
    }
    double x = 0;
    double gamma_max = gamma_[gamma_.size() - 2];
    double guess = 0.5 * (gamma_.front() + gamma_max);  // second to the last is 1.9
    boost::uintmax_t maxit = 20;
    double gamma_opt =
        boost::math::tools::newton_raphson_iterate(fourthFunctorDeriv(x), guess, gamma_.front(), gamma_max, 7, maxit);

    double f1 = poly(coff, fit_order, gamma_.front());
    double f2 = poly(coff, fit_order, gamma_max);
    double f_opt = poly(coff, fit_order, gamma_opt);
    if (f_opt < f1 || f_opt < f2)
    {
      gamma_opt = f1 > f2 ? gamma_.front() : gamma_max;
    }

    float exposure_ratio = updateExposureRatio(gamma_opt);
    // set shutter speed and gain based on exposure ratio
    updateGainShutter(exposure_ratio);

#if DEBUG
    ros::Time t5 = ros::Time::now();
    std::cout << "get max gamma and update exposure ratio: " << (t5 - t4).toSec() * 1000 << " ms" << std::endl;
    std::cout << "total time: " << (t5 - t1).toSec() * 1000 << " ms" << std::endl << std::endl;
#endif
  }

  boost::shared_ptr<boost::thread> imgGainShutterThread_;  // obtain gain and shutter thread

if (!config_.auto_gain && !config_.auto_shutter)
            {
              imgGainShutterThread_.reset(new boost::thread(
                  boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::setGainShutter, this)));
            }
double gain_max_;
  double shutter_speed_max_;
volatile double gain_;
  volatile double shutter_;
```

