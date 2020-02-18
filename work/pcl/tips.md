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
