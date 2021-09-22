import os

pcd_file_dir =  "/home/tan/data/database_1/baidu_lidar_labeling/pcds"
saved_bin_file_dir =  "/home/tan/data/database_1/baidu_lidar_labeling/pcds"
pcd_file_list = os.listdir(pcd_file_dir)

for file_name in pcd_file_list:
    if '.pcd' in file_name:
        pcd_file = os.path.join(pcd_file_dir, file_name)
        output_bin_file = os.path.join(saved_bin_file_dir, file_name.replace('.pcd', '.bin'))
        # print(pcd_file, output_bin_file)
        os.system('/home/tan/new_workspace/3DShow/build/pcd2bin --infile %s --outfile %s' % (pcd_file, output_bin_file))
