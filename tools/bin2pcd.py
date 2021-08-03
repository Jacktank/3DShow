import os

bin_file_dir =  "/home/tan/Downloads/1_2000/pcl"
saved_pcd_file_dir =  "/home/tan/Downloads/1_2000/pcd_file"
bin_file_list = os.path.join(bin_file_dir, "list.txt")

with open(bin_file_list) as file:
    for line in file:
        bin_file = os.path.join(bin_file_dir, line.strip())
        output_pcd_file = os.path.join(saved_pcd_file_dir, line.strip().split('.')[0] +'.pcd')
        # print(bin_file, output_pcd_file)
        os.system('/home/tan/workspace/lidar_test/build/bin2pcd --infile %s --outfile %s' % (bin_file, output_pcd_file))
