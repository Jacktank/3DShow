import os
import glob
import math
# import random
import numpy as np
import open3d as o3d

def cls_type_to_id(cls_type):
    type_to_id = {'Unknown': 0}
    if cls_type not in type_to_id.keys():
        return -1
    return type_to_id[cls_type]

class Object3d(object):
    def __init__(self, line):
        label = line.strip().split(' ')
        self.src = line
        self.cls_type = label[0].lower()
        self.cls_id = cls_type_to_id(self.cls_type)
        self.score = float(label[1])
        self.center = np.array((float(label[2]), float(label[3]), float(label[4])), dtype=np.float32)
        self.len = float(label[5])
        self.width = float(label[6])
        self.height = float(label[7])
        self.heading = float(label[8])
        self.num_pts = 100000
        self.tight_box_center = self.center
        self.tight_box_size = [self.len, self.width, self.height]
        if len(label) > 9:
            self.num_pts = float(label[9])
            self.tight_box_center = [float(label[10]), float(label[11]), float(label[12])]
            self.tight_box_size = [float(label[13]), float(label[14]), float(label[15])]

class TemplateO3DShow():
    def __init__(self, pcd_path, pred_path, gt_path=None):
        self.pcd_path = pcd_path
        self.pred_path = pred_path
        self.gt_path = gt_path

        self.gt_datas = []
        self.pred_datas = []

        self.include_infos()

    def include_infos(self):
        self.gt_infos()
        self.pred_infos()
        if self.gt_path is not None and self.pred_path is not None:
            assert len(self.gt_datas) == len(self.pred_datas)

    def gt_infos(self):
        self.gt_datas = []
        return self.gt_datas

    def pred_infos(self):
        self.pred_datas = []
        return self.pred_datas

count_count_ = 0

class O3DShow(TemplateO3DShow):
    def __init__(self, pcd_path, pred_path, class_name = ['car', 'pedestrian', 'cyclist', 'truck', 'bus'], 
                    gt_path=None, capture_save_path=None, pcd_ext=".pcd", exclude_class_name = None):
        super().__init__(pcd_path, pred_path, gt_path)

        self.class_name = class_name

        self.pcd_ext = pcd_ext
        self.exclude_class_name = exclude_class_name

        self.vis = None
        self.begin_vis_flag = True
        self.pcd_for_render = None
        self.boxes_for_render = []
        self.colors_palette = {'car': (0, 0, 1), 'cyclist': (0, 1, 0), 'pedestrian': (1, 0, 0),
                                'bus': (1, 1, 0), 'truck': (0, 1, 1), 'mini_truck': (0, 1, 1), 'construction': (1, 0, 1),
                                'tricycle': (1, 1, 1), 'bicycle':(0.25, 0.5, 0.25), 'motorcyclist':(0.25, 0.5, 0.25),
                                'tricyclist':(0.25, 0.5, 0.25), 'small_movable': (0.5, 0, 0.5), 'small_unmovable': (0.5, 0, 0.5),
                                'van': (0.25, 0.25, 0), 'blur': (0.5, 0.5, 0), 'noise': (0.5, 0.5, 0),
                                'construction_sign': (0.5, 0, 0.5), 'traffic_cone': (0.5, 0, 0.5), 'crash_barrel': (0.5, 0, 0.5),
                                'special_vehicle': (0.5, 0, 0.5), 'water_horse': (0.5, 0, 0.5), 'other': (0.5, 0, 0.5)}

        dist_per_count = 30.0
        self.show_distance_dict = {"dist_count": 0, "max_dist": 240.0, "dist_per_count": dist_per_count, "rendered_circle_geos": []}
        self.show_distance_dict['dist_show_list'] = [dist_per_count * (i + 1) for i in range(int(self.show_distance_dict['max_dist'] / dist_per_count))]

        self.frame_idx = 0
        self.capture_save_path = capture_save_path
        self.capture_frame_count = 0

    def gt_infos(self):
        if len(self.gt_datas) != 0:
            return self.gt_datas

        if self.gt_path is not None:
            self.gt_datas = []

        return self.gt_datas

    def pred_infos(self):
        if len(self.pred_datas) != 0:
            return self.pred_datas

        if self.pred_path is not None:
            self.pred_datas = glob.glob(os.path.join(self.pred_path, '*.txt'))
            # self.pred_datas.sort()
            # random.shuffle(self.pred_datas)
        return self.pred_datas


    def create_draw_windows_with_key_callback(self):
        def create_circle(radius=10.0, sparse=360, color=(1, 0, 0)):
            circle_angle = np.linspace(0, 2*np.pi, sparse)
            x = radius*np.cos(circle_angle)
            y = radius*np.sin(circle_angle)
            z = np.zeros_like(x)

            num_pts = x.shape[0]

            points = [[x[i], y[i], z[i]] for i in range(num_pts)]
            lines = [[i, i+1] for i in range(num_pts - 1)]
            colors = [list(color) for i in range(len(lines))]
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(colors)
            return line_set

        def key_callback_save_screen(vis):
            if self.capture_save_path is None:
                return

            vis.poll_events()
            vis.update_renderer()
            # Capture image
            pred_file_full_name = self.pred_datas[self.frame_idx-1]
            pred_file_name = pred_file_full_name.split('/')[-1].split('.')[0]
            save_image_name = os.path.join(self.capture_save_path, str(pred_file_name) + '_' + str(self.capture_frame_count)) + '.png'
            vis.capture_screen_image(save_image_name)
            print('capture screen image to ', save_image_name)
            self.capture_frame_count += 1

        def key_callback_draw_circle(vis):
            self.show_distance_dict['dist_count'] += 1
            if self.show_distance_dict['dist_count'] > len(self.show_distance_dict['dist_show_list']):
                self.show_distance_dict['dist_count'] = 0
            key_callback(vis)

        def key_callback_forward(vis):
            key_callback(vis)
            self.frame_idx += 1

        def key_callback_fast_forward(vis):
            key_callback(vis)
            self.frame_idx += 10

        def key_callback_backward(vis):
            key_callback(vis)
            self.frame_idx -= 1

        def key_callback_fast_backward(vis):
            key_callback(vis)
            self.frame_idx -= 10

        def key_callback(vis):
            if self.frame_idx >= len(self.pred_datas):
                self.frame_idx = 0
            if self.frame_idx < 0:
                self.frame_idx = len(self.pred_datas) - 1

            # vis.clear_geometries() for boxes
            for geo in self.boxes_for_render:
                vis.remove_geometry(geo, False)
            self.boxes_for_render.clear()
            # vis.clear_geometries() for circle_distance
            for geo in self.show_distance_dict['rendered_circle_geos']:
                vis.remove_geometry(geo, False)
            self.show_distance_dict['rendered_circle_geos'].clear()

            pred_file_full_name = self.pred_datas[self.frame_idx]
            pcd_file_name = pred_file_full_name.split('/')[-1].split('.')[0] + self.pcd_ext

            pcd_file_full_name = os.path.join(self.pcd_path, pcd_file_name)
            
            print('-------------------- frame idx: ', self.frame_idx, ' --------------------')
            print('pcd_file_name: ', pcd_file_name)
            if os.path.exists(pcd_file_full_name) == False:
                return

            # load pcd file
            if self.pcd_ext == '.bin':
                pcd_data = np.fromfile(pcd_file_full_name, dtype=np.float32).reshape(-1, 4)
                pcd_data[:, 3] = 0
            elif self.pcd_ext == '.npy':
                pcd_data = np.load(pcd_file_full_name)
            elif self.pcd_ext == '.pcd':
                pcd = o3d.io.read_point_cloud(pcd_file_full_name)
                points_ori = np.asarray(pcd.points)
                pcd_data = np.zeros(shape=(points_ori.shape[0], 4), dtype=points_ori.dtype)
                pcd_data[:, :3] = points_ori
            else:
                raise NotImplementedError
            
            # add geometry: pcd
            self.pcd_for_render.points = o3d.utility.Vector3dVector(pcd_data[:, :3])
            colors = np.ones_like(pcd_data[:, :3])
            self.pcd_for_render.colors = o3d.utility.Vector3dVector(colors)

            if self.begin_vis_flag:
                vis.clear_geometries()
                vis.add_geometry(self.pcd_for_render)
                self.begin_vis_flag = False
            else:
                vis.update_geometry(self.pcd_for_render)

            for circle_count in range(self.show_distance_dict['dist_count']):
                circle_geo = create_circle(radius=self.show_distance_dict['dist_show_list'][circle_count])
                vis.add_geometry(circle_geo, False)
                self.show_distance_dict['rendered_circle_geos'].append(circle_geo)

            # load predict file
            pred_objs = []
            print(pred_file_full_name)
            with open(pred_file_full_name, 'r') as pred_file:
                lines = pred_file.readlines()
                pred_objs = [Object3d(line) for line in lines]

            # parse pred result
            print('pred_box nums: ', len(pred_objs))

            global count_count_
            count_count_ += len(pred_objs)
            print(count_count_)
            for box_idx in range(len(pred_objs)):
                obj = pred_objs[box_idx]

                cls_type = obj.cls_type
                x, y, z = obj.center
                xlen = obj.len
                ylen = obj.width
                zlen = obj.height
                heading = obj.heading

                # optional
                num_pts = obj.num_pts
                tight_box_center = obj.tight_box_center
                tight_box_size = obj.tight_box_size

                points = np.array([[0, 0, 0], [xlen, 0, 0], [0, ylen, 0], [xlen, ylen, 0], [0, 0, zlen], [xlen, 0, zlen],
                    [0, ylen, zlen], [xlen, ylen, zlen]]).astype(np.float64)
                lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
                        [0, 4], [1, 5], [2, 6], [3, 7]]
                center = np.asarray([xlen, ylen, zlen]).astype(np.float64) * 0.5
                colors = [list(self.colors_palette.get(cls_type, (1, 0, 0))) for i in range(len(lines))]

                # colors = [color_t for i in range(len(lines))]
                points = points - center
                transform = np.array([[math.cos(heading), -math.sin(heading), 0, x],
                                        [math.sin(heading), math.cos(heading), 0, y],
                                        [0, 0, 1, z], [0, 0, 0, 1]], dtype=np.float64)

                line_set = o3d.geometry.LineSet()
                line_set.points = o3d.utility.Vector3dVector(points)
                line_set.lines = o3d.utility.Vector2iVector(lines)
                line_set.colors = o3d.utility.Vector3dVector(colors)
                line_set.transform(transform)

                # add geometry: box lines and corners
                vis.add_geometry(line_set, False)
                self.boxes_for_render.append(line_set)

            vis.poll_events()
            vis.update_renderer()

            if False:
                key_callback_save_screen(vis)

        self.vis = o3d.visualization.VisualizerWithKeyCallback()

        # Enter 'f' key to show forward frame
        # Enter 'b' key to show backward frame
        # Enter 'j' key to show fast forward frame
        # Enter 'k' key to show fast backward frame
        # Enter 'Esc' key to exit show
        # Enter 's' key to capture screen image
        self.vis.register_key_callback(ord('F'), key_callback_forward)
        self.vis.register_key_callback(ord('B'), key_callback_backward)
        self.vis.register_key_callback(ord('J'), key_callback_fast_forward)
        self.vis.register_key_callback(ord('K'), key_callback_fast_backward)
        self.vis.register_key_callback(ord('S'), key_callback_save_screen)
        self.vis.register_key_callback(ord('C'), key_callback_draw_circle)
        self.vis.create_window()

        render_opt = self.vis.get_render_option()
        render_opt.show_coordinate_frame = True
        render_opt.background_color = np.asarray([0, 0, 0])
        render_opt.point_size = 1.0

        self.vis.update_renderer()

        self.pcd_for_render = o3d.geometry.PointCloud()
        self.pcd_for_render.points = o3d.utility.Vector3dVector(np.random.rand(1000, 3))

        self.vis.add_geometry(self.pcd_for_render)
        self.vis.run()

pcd_path = "./data/test_pcds/"
pred_path = "./data/pred_res"
capture_save_path = "./data/capture_images"

handle_show = O3DShow(pcd_path=pcd_path, pred_path=pred_path, capture_save_path=capture_save_path, pcd_ext=".bin")

handle_show.create_draw_windows_with_key_callback()
