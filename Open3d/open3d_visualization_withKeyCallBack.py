import numpy as np
import open3d as o3d

vis = o3d.visualization.VisualizerWithKeyCallback()

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.random.rand(10000, 3))

count = 0

def key_callback(vis):
    global count
    count += 1
    print(count)

    vis.clear_geometries()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.random.rand(10000, 3))
    vis.add_geometry(pcd)
    
    vis.poll_events()
    vis.update_renderer()
    return True

# Enter space key to show new frame
# Enter 'Esc' key to exit show
vis.register_key_callback(ord(' '), key_callback)

vis.create_window()
render_opt = vis.get_render_option()
render_opt.background_color = np.asarray([0, 0, 0])
render_opt.point_size = 1.5
vis.update_renderer()

vis.add_geometry(pcd)
vis.run()

