#include <iostream>
#include <fstream>
#include <gflags/gflags.h>
#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

struct MyPointType
{
  float x;
  float y;
  float z;
  float intensity;
  double timestamp;
  u_short ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (double, timestamp, timestamp)
                                   (u_short, ring, ring)
)

pcl::PointCloud<MyPointType>::Ptr basic_cloud_ptr(new pcl::PointCloud<MyPointType>);

DEFINE_string(pcd_root_dir, "/home/tan/data/lidar_pose/Pandar_20210129-114549_621_2/", "src path of point cloud data files");
DEFINE_string(dst_dir, "/home/tan/data/lidar_pose/Pandar_20210129-114549_621_2/", "dst path of point cloud data files");

int main (int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    int pcd_count = 0;
    int max_pcd_nums = 0;
    std::vector<std::string> pcd_files_list;

    std::string pcd_list_file = FLAGS_pcd_root_dir + "list.txt";
    std::ifstream file_in(pcd_list_file, ios::out);
    if(!file_in) { 
        std::cerr<<"Can't open the file."<<endl; 
        return -1; 
    } 
    std::string line;
    while(getline(file_in, line)) 
        pcd_files_list.push_back(FLAGS_pcd_root_dir + line); 
    max_pcd_nums = pcd_files_list.size();
    file_in.close();
    
    
    if (false) {
        if (pcl::io::loadPCDFile<MyPointType> (pcd_files_list[pcd_count], *basic_cloud_ptr) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            std::cout << "Couldn't read file " << pcd_files_list[pcd_count] << std::endl;
            return -1;
        } else {
            std::cout << "Succeed " << std::endl;
        }

        std::cout << pcd_files_list[pcd_count] << std::endl;
        std::cout << basic_cloud_ptr->points.size() << std::endl;
        std::cout << basic_cloud_ptr->is_dense << std::endl;
        std::cout << basic_cloud_ptr->header << std::endl;
        for (const auto& pt : basic_cloud_ptr->points) {
            if (pt.x > 100.0) {
                std::cout << std::setprecision(64) << pt.x << ", " << pt.y << ", " << pt.z << ", " << std::to_string(pt.intensity) << ", " << pt.timestamp << ", " << std::to_string(pt.ring) << std::endl;
            }
        }
    }
    
    if (true) {
        for(int count = 0; count < max_pcd_nums; count++) {
            if (pcl::io::loadPCDFile<MyPointType> (pcd_files_list[count], *basic_cloud_ptr) == -1) {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                std::cout << "Couldn't read file " << pcd_files_list[count] << std::endl;
                return -1;
            } else {
                std::cout << "Succeed " << std::endl;
            }

            std::cout << pcd_files_list[count] << std::endl;
            std::cout << basic_cloud_ptr->points.size() << std::endl;
            std::cout << basic_cloud_ptr->is_dense << std::endl;
            std::cout << basic_cloud_ptr->header << std::endl;

            // load point cloud
            std::vector<std::string> fields1;
            boost::split(fields1, pcd_files_list[count], boost::is_any_of("/"));
            std::vector<std::string> fields2;
            boost::split(fields2, fields1[fields1.size() - 1], boost::is_any_of("."));

            std::string outfile = FLAGS_dst_dir + "/" + fields2[0] + ".bin";
            std::cout << outfile << std::endl;
            std::fstream bin_file(outfile, std::ios::out | std::ios::binary);
            for (size_t i = 0; i < basic_cloud_ptr->points.size (); ++i)
            {
                // bin_file<<std::setprecision(5);
                bin_file.write((char*)&basic_cloud_ptr->points[i].x, 3*sizeof(float)); 
                bin_file.write((char*)&basic_cloud_ptr->points[i].intensity, sizeof(float));
                // std::cout<< basic_cloud_ptr->points[i].x << ", " << basic_cloud_ptr->points[i].y << ", " << basic_cloud_ptr->points[i].z << ", " << basic_cloud_ptr->points[i].intensity << endl;
            }
            bin_file.close();
        }
    }
    
    
    return (0);
}
