# PCL Functions

### 计算两点之间的欧式平方距离（不开方）
```cpp
#include <pcl/common/distances.h>
template <typename PointType1, typename PointType2> inline float 
squaredEuclideanDistance (const PointType1& p1, const PointType2& p2)
{
   float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
   return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
}
```

### 计算2点欧式距离
```cpp
#include <pcl/common/distances.h>
00647 template <typename PointType1, typename PointType2> inline float 
00648 euclideanDistance (const PointType1& p1, const PointType2& p2)
00649 {
00650   return (sqrtf (squaredEuclideanDistance (p1, p2)));
00651 }
```

### 判断point的x, y, z是否为finite
```cpp
#include <pcl/common/point_tests.h>
pcl::isFinite(point)
pcl_isfinite(point.x)
```

### pointXYZGRB中rgb解析
```cpp
// pack r/g/b into rgb
uint8_t r = 255, g = 0, b = 0;    // Example: Red color
uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
p.rgb = *reinterpret_cast<float*>(&rgb);
PointXYZRGB p;
// unpack rgb into r/g/b
uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
uint8_t r = (rgb >> 16) & 0x0000ff;
uint8_t g = (rgb >> 8)  & 0x0000ff;
uint8_t b = (rgb)       & 0x0000ff;
```

### `#include <pcl/registration/distance.h>中的函数`
```cpp
求array中的中值
        /** \brief Compute the median value from a set of doubles
   52       * \param[in] fvec the set of doubles
   53       * \param[in] m the number of doubles in the set
   54       */
   55     inline double 
   56     computeMedian (double *fvec, int m)
   57     {
   58       // Copy the values to vectors for faster sorting
   59       std::vector<double> data (m);
   60       memcpy (&data[0], fvec, sizeof (double) * m);
   61       
   62       std::nth_element(data.begin(), data.begin() + (data.size () >> 1), data.end());
   63       return (data[data.size () >> 1]);
   64     }

马氏距离
  113     /** \brief Compute the Manhattan distance between two eigen vectors.
  114       * \param[in] p_src the first eigen vector
  115       * \param[in] p_tgt the second eigen vector
  116       */
  117     inline double
  118     l1 (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt) 
  119     {
  120       return ((p_src.array () - p_tgt.array ()).abs ().sum ());
  121     }

欧式距离
  123     /** \brief Compute the Euclidean distance between two eigen vectors.
  124       * \param[in] p_src the first eigen vector
  125       * \param[in] p_tgt the second eigen vector
  126       */
  127     inline double
  128     l2 (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt) 
  129     {
  130       return ((p_src - p_tgt).norm ());
  131     }

欧式平方距离
  133     /** \brief Compute the squared Euclidean distance between two eigen vectors.
  134       * \param[in] p_src the first eigen vector
  135       * \param[in] p_tgt the second eigen vector
  136       */
  137     inline double
  138     l2Sqr (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt) 
  139     {
  140       return ((p_src - p_tgt).squaredNorm ());
  141     }
  142   }
```

### 点到直线的距离
```cpp
#include <pcl/common/distances.h>
  double inline
  sqrPointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
    return (line_dir.cross3 (line_pt - pt)).squaredNorm () / line_dir.squaredNorm ();
  }
  /** \brief Get the square distance from a point to a line (represented by a point and a direction)
    * \note This one is useful if one has to compute many distances to a fixed line, so the vector length can be pre-computed
    * \param pt a point
    * \param line_pt a point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
    * \param line_dir the line direction
    * \param sqr_length the squared norm of the line direction
    * \ingroup common
    */
  double inline
  sqrPointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir, const double sqr_length)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
    return (line_dir.cross3 (line_pt - pt)).squaredNorm () / sqr_length;
  }
```

### 点到平面的距离
距离的平方
```cpp
inline float sqrPointToPlaneDistance(const pcl::PointXYZI &in_point, const Eigen::VectorXf &plane_model)
{
  Eigen::Vector4f tmp_v4f(in_point.x, in_point.y, in_point.z, 1);
  float numerator = plane_model.dot(tmp_v4f);
  return numerator * numerator / plane_model.head(3).squaredNorm();
}
```
### 快速复制点云
`memcpy(&out_cloud_ptr->points[0], &in_cloud_ptr->points[0], point_num * sizeof(pcl::PointXYZI));`

### angels.hpp
```cpp
#include <pcl/common/angles.h>
(1) normAngle
/** \brief Normalize an angle to (-PI, PI]
    * \param alpha the input angle (in radians)
    * \ingroup common
    */
inline float
normAngle (float alpha)
{
    return (alpha >= 0 ? fmodf (alpha + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI)) - static_cast<float>(M_PI) 
           : 
 -(fmodf (static_cast<float>(M_PI) - alpha, 2.0f * static_cast<float>(M_PI)) - static_cast<float>(M_PI)));
}

(2) rad2deg
/** \brief Convert an angle from radians to degrees
  * \param alpha the input angle (in radians)
  * \ingroup common
  */
inline float 
rad2deg (float alpha)
{
    return (alpha * 57.29578f);
}

(3) deg2rad
/** \brief Convert an angle from degrees to radians
  * \param alpha the input angle (in degrees)
  * \ingroup common
  */
inline float 
deg2rad (float alpha)
{
  return (alpha * 0.017453293f);
}
```