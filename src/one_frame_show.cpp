#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Please input the path of *.bin file" << std::endl;
        return -1;
    }
    // input the path of *.bin file
    std::string pcd_root_file(argv[1]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_root_file, *cloud) == -1) //* load the file 
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
     
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    
    while (!viewer.wasStopped())
    {
    }

    return (0);
}