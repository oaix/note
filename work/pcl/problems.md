# Problem

1. 旋转矩阵
```cpp
Eigen::Affine3f tt;
    for (std::size_t i = 0; i < 4; ++i)
    {
      for (std::size_t j = 0; j < 4; ++j)
      {
        tt(i, j) = static_cast<float>(transform_matrix_(i, j));
      }
    }
    float x, y, z, roll, pitch, yaw;
//pcl::getTranslationAndEulerAngles由旋转矩阵得到的姿态角是安装xyz的顺序
    pcl::getTranslationAndEulerAngles(tt, x, y, z, roll, pitch, yaw);
    std::cout << "pcl get euler angles: " << roll << ", " << pitch << ", " << yaw << std::endl;
    Eigen::Quaterniond ag = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    std::cout << "pcl t matrix: " << std::endl << ag.matrix() << std::endl;


    Eigen::Matrix3d rotation = transform_matrix_.topLeftCorner(3, 3);
// rotation.eulerAngles(0, 1, 2)得到的姿态角是zyx的顺序,只要按照获取姿态角的顺序生成旋转矩阵即可
    Eigen::Vector3d euler_angle = rotation.eulerAngles(0, 1, 2);

x_ = transform_matrix_(0, 3);
    y_ = transform_matrix_(1, 3);
    z_ = transform_matrix_(2, 3);
    roll_ = euler_angle(0);
    pitch_ = euler_angle(1);
    yaw_ = euler_angle(2);

    std::cout << "eigen euler angles: " << roll_ << ", " << pitch_ << ", " << roll_ << std::endl;
    Eigen::Translation3d translation(x_, y_, z_);
    Eigen::AngleAxisd rotation_x(roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotation_y(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotation_z(yaw_, Eigen::Vector3d::UnitZ());
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tt_eigen =
        (translation * rotation_x * rotation_y * rotation_z).matrix();
    std::cout << "eigen t matrix: " << std::endl << tt_eigen << std::endl;
```

