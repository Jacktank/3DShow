#include <iostream>
#include <fstream>
#include <gflags/gflags.h>
#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int pcd_count = 0;
int max_pcd_nums = 0;
std::vector<std::string> pcd_files_list;

struct MyPointType
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float time_stamp;
  float ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, time_stamp, time_stamp)
                                   (float, ring, ring)
)
  

pcl::PointCloud<pcl::PointXYZI>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> custom_color(basic_cloud_ptr, 0, 255, 0);

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
  if (event.getKeySym () == "n" && event.keyDown ()) {
    std::cout << "n was pressed => update point cloud: " << pcd_files_list[pcd_count] << std::endl;
    //* load the file 
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_files_list[pcd_count], *basic_cloud_ptr) == -1) {
        PCL_ERROR ("Couldn't read file pcd \n");
        std::cout << "Couldn't read file " << pcd_files_list[pcd_count] << std::endl;
        return;
    } else {
        std::cout << pcd_files_list[pcd_count] << std::endl;
        viewer->removePointCloud();
        viewer->addPointCloud(basic_cloud_ptr, custom_color);
    }
    pcd_count++;
    if (pcd_count >= max_pcd_nums) pcd_count = 0;
  }
}

void keyboardEventOccurred_custom_load(const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
  if (event.getKeySym () == "n" && event.keyDown ()) {
    std::cout << "n was pressed => update point cloud: " << pcd_files_list[pcd_count] << std::endl;
    //* load the file 
	
    std::fstream input(pcd_files_list[pcd_count], std::ios::in | std::ios::binary);
    if(!input.good()){
      std::cerr << "Could not read file: " << pcd_files_list[pcd_count] << std::endl;
      std::exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
      pcl::PointXYZI point;
      float ring;
      input.read((char *) &point.x, 3*sizeof(float));
      input.read((char *) &point.intensity, sizeof(float));
      // input.read((char *) &ring, sizeof(float));
      basic_cloud_ptr->push_back(point);
      // std::cout << point.x << ", " << point.y << ", " << point.z << ", " << point.intensity << std::endl;
    }
	  input.close();
 
    std::cout << pcd_files_list[pcd_count] << std::endl;
    viewer->removePointCloud();
    viewer->addPointCloud(basic_cloud_ptr, custom_color);
    basic_cloud_ptr->clear();

    pcd_count++;

    if (pcd_count >= max_pcd_nums) pcd_count = 0;
  }
}

pcl::visualization::PCLVisualizer::Ptr interactionCustomizationVis()
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  // 定制颜色

  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

  return (viewer);
}

DEFINE_string(pcd_root_dir, "/home/tan/data/lidar_pose/Pandar_20210129-114549_621_2/", "path of point cloud data files");

int main (int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::string pcd_list_file = FLAGS_pcd_root_dir + "list.txt";
    ifstream file_in(pcd_list_file, ios::out);
    if(!file_in) { 
        std::cerr<<"Can't open the file."<<endl; 
        return -1; 
    } 
    std::string line;
    while(getline(file_in, line)) 
        // pcd_files_list.push_back(FLAGS_pcd_root_dir + line); 
        pcd_files_list.push_back(line); 
    max_pcd_nums = pcd_files_list.size();
    file_in.close();
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    // viewer = simpleVis(basic_cloud_ptr);
    viewer = interactionCustomizationVis();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce();
        // viewer->spinOnce(100);
        // std::this_thread::sleep_for(100ms);
    }
   
    return (0);
}
