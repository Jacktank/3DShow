#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
 
#include <iostream>
#include <fstream>
 
using namespace pcl;
using namespace std;
 
struct MyPointType
{
  float x;
  float y;
  float z;
  u_char intensity;
  double timestamp;
  u_short ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (u_char, intensity, intensity)
                                   (double, timestamp, timestamp)
                                   (u_short, ring, ring)
)

namespace po = boost::program_options;
 
int main(int argc, char **argv){
	///The file to read from.
	string infile;
 
	///The file to output to.
	string outfile;
 
	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Options
		("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
		("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normals to")
		;
	// Parse the command line
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
 
	// Print help
	if (vm.count("help"))
	{
		cout << desc << "\n";
		return false;
	}
 
	// Process options.
	po::notify(vm);
 
	// load point cloud
	fstream bin_file(outfile, ios::out | ios::binary);
	
	pcl::PointCloud<MyPointType>::Ptr basic_cloud_ptr(new pcl::PointCloud<MyPointType>);
	if (pcl::io::loadPCDFile<MyPointType> (infile, *basic_cloud_ptr) == -1) {
		std::cout << "Couldn't read file " << infile << std::endl;
		return -1;
	}
 
	for (size_t i = 0; i < basic_cloud_ptr->points.size (); ++i)
	{
		// bin_file<<std::setprecision(5);
        bin_file.write((char*)&basic_cloud_ptr->points[i].x, sizeof(float));
        bin_file.write((char*)&basic_cloud_ptr->points[i].y, sizeof(float));
        bin_file.write((char*)&basic_cloud_ptr->points[i].z, sizeof(float));

       	bin_file.write((char*)&basic_cloud_ptr->points[i].intensity, sizeof(u_char));
        // std::cout<< basic_cloud_ptr->points[i].x << ", " << basic_cloud_ptr->points[i].y << ", " << basic_cloud_ptr->points[i].z << ", " << basic_cloud_ptr->points[i].intensity << endl;
	}
	bin_file.close();
    return 0;
}