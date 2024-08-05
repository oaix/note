1. [保存pcd为binary比ASCII要减少60%的存储空间](http://docs.pointclouds.org/1.9.1/classpcl_1_1_p_c_d_writer.html)

   + binary

     ```cpp
     #include <pcl/io/pcd_io.h>  
     pcl::io::savePCDFileBinary(map_file, map_cloud);
     template<typename PointT >
     int pcl::io::savePCDFile(const std::string & file_name,
                              const pcl::PointCloud< PointT > & 	cloud,
                              bool 	binary_mode = false 
     )
     
     
     ```


   + ASCII

     ```cpp
     emplate<typename PointT >
     int pcl::io::savePCDFileASCII	(	const std::string & 	file_name,
     const pcl::PointCloud< PointT > & 	cloud 
     )	
     ```

2. [How to add a new PointT type](https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html#id5)

   pcl template class/function实例化了所有已经定义的point类型(用户调用时，无需再次编译，加速编译过程)，如果自定义新的PointT，需要在cpp顶部添加:

   ```cpp
   #define PCL_NO_PRECOMPILE
   ```

   让pcl使用PointT实例化template class/function，并编译；这样PointT才可调用pcl库class/function.
