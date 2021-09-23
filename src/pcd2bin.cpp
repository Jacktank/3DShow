#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <boost/algorithm/string.hpp>

#include <fstream>
#include <chrono>    // std::chrono::seconds
#include <iostream>  // std::cout
#include <thread>    // std::thread, std::this_thread::sleep_for

 
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

void convert_pcd_to_bin(int n, int num_threads, std::vector<std::string> lines, std::string output_dir) {
	int num_line_per_thread = lines.size() / num_threads + 1;
	int aaa = (n - 1) * num_line_per_thread;
	int bbb = n * num_line_per_thread;
	bbb = lines.size() - 1 < bbb ? lines.size() - 1: bbb;
	std::this_thread::sleep_for(std::chrono::seconds(n));
	std::cout << "hello thread "
        << std::this_thread::get_id()
        << " paused " << n << " seconds, " << lines.size() << ", " << aaa  << ", " << bbb << std::endl;
	
	for (int i = aaa; i < bbb; i++) {
		std::string input_file_name = lines[i];

		std::vector<std::string> vecSegTag;  
		boost::split(vecSegTag, input_file_name, boost::is_any_of("/"));
		std::string pure_name = vecSegTag.back();
		vecSegTag.clear();
		boost::split(vecSegTag, pure_name, boost::is_any_of("."));
		std::string new_pure_name = vecSegTag[0] + ".bin";
		std::string output_file_name = output_dir + "/" + new_pure_name;

		std::cout << input_file_name << std::endl;
		std::cout << output_file_name << std::endl;

		// load point cloud
		std::fstream bin_file(output_file_name, ios::out | ios::binary);
		
		pcl::PointCloud<MyPointType>::Ptr basic_cloud_ptr(new pcl::PointCloud<MyPointType>);
		if (pcl::io::loadPCDFile<MyPointType> (input_file_name, *basic_cloud_ptr) == -1) {
			std::cout << "Couldn't read file " << input_file_name << std::endl;
			continue;
		}

		for (size_t i = 0; i < basic_cloud_ptr->points.size (); ++i)
		{
			// bin_file<<std::setprecision(5);
			bin_file.write((char*)&basic_cloud_ptr->points[i].x, sizeof(float));
			bin_file.write((char*)&basic_cloud_ptr->points[i].y, sizeof(float));
			bin_file.write((char*)&basic_cloud_ptr->points[i].z, sizeof(float));

			float intensity_float = float(basic_cloud_ptr->points[i].intensity);
			bin_file.write((char*)&intensity_float, sizeof(float));
			// std::cout<< basic_cloud_ptr->points[i].x << ", " << basic_cloud_ptr->points[i].y << ", " 
			// 			<< basic_cloud_ptr->points[i].z << ", " << std::to_string(basic_cloud_ptr->points[i].intensity) << endl;
		}
		bin_file.close();
	}

}

int main(int argc, char **argv){
	///The file to read from.
	string infile;
 
	///The file to output to.
	string output_dir;

	///The number of threads
	int num_threads;
 
	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Options
		("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
		("output_dir", po::value<string>(&output_dir)->required(), "the file to write the DoN point cloud & normals to")
		("num_threads", po::value<int>(&num_threads)->required(), "The number of threads to")
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

    std::ifstream infile_f; 
    infile_f.open(infile);   //将文件流对象与文件连接起来 
    assert(infile_f.is_open());   //若失败,则输出错误消息,并终止程序运行 

	std::vector<std::string> vec_lines;
    string s;
    while(getline(infile_f,s))
    {
		vec_lines.push_back(s);
    }
    infile_f.close();             //关闭文件输入流 

	if (num_threads > 0) {
		std::thread threads[num_threads];
		std::cout << "Spawning num_threads threads...\n";
		for (int i = 0; i < num_threads; i++) {
			threads[i] = std::thread(convert_pcd_to_bin, i + 1, num_threads, vec_lines, output_dir);
		}
		std::cout << "Done spawning threads! Now wait for them to join\n";
		for (auto& t: threads) {
			t.join();
		}
		std::cout << "All threads joined.\n";
	} else {
		std::cout << "Disable multi threads mode " << std::endl;

		for (const auto& input_file_name : vec_lines) {
			std::cout << "processing " << input_file_name << std::endl;
			
			std::vector<std::string> vecSegTag;  
			boost::split(vecSegTag, input_file_name, boost::is_any_of("/"));
			std::string pure_name = vecSegTag.back();
			vecSegTag.clear();
			boost::split(vecSegTag, pure_name, boost::is_any_of("."));
			std::string new_pure_name = vecSegTag[0] + ".bin";
			std::string output_file_name = output_dir + "/" + new_pure_name;

			std::cout << input_file_name << std::endl;
			std::cout << output_file_name << std::endl;

			// load point cloud
			fstream bin_file(output_file_name, ios::out | ios::binary);
			
			pcl::PointCloud<MyPointType>::Ptr basic_cloud_ptr(new pcl::PointCloud<MyPointType>);
			if (pcl::io::loadPCDFile<MyPointType> (input_file_name, *basic_cloud_ptr) == -1) {
				std::cout << "Couldn't read file " << input_file_name << std::endl;
				continue;
			}

			for (size_t i = 0; i < basic_cloud_ptr->points.size (); ++i)
			{
				// bin_file<<std::setprecision(5);
				bin_file.write((char*)&basic_cloud_ptr->points[i].x, sizeof(float));
				bin_file.write((char*)&basic_cloud_ptr->points[i].y, sizeof(float));
				bin_file.write((char*)&basic_cloud_ptr->points[i].z, sizeof(float));

				float intensity_float = float(basic_cloud_ptr->points[i].intensity);
				bin_file.write((char*)&intensity_float, sizeof(float));
				// std::cout<< basic_cloud_ptr->points[i].x << ", " << basic_cloud_ptr->points[i].y << ", " 
				// 		<< basic_cloud_ptr->points[i].z << ", " << std::to_string(basic_cloud_ptr->points[i].intensity) << endl;
			}
			bin_file.close();
		}
	}

	
    return 0;
}