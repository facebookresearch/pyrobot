from pyrobot import Robot

import os
import open3d


# Please change this to match your habitat_sim repo's path
path_to_habitat_scene = os.path.dirname(os.path.realpath(__file__))
relative_path = "scenes/skokloster-castle.glb"

common_config = dict(scene_path=os.path.join(path_to_habitat_scene, relative_path))
bot = Robot("habitat", common_config=common_config)

# fetch the point
pts, colors = bot.camera.get_current_pcd(in_cam=False)

# convert points to open3d point cloud object
pcd = open3d.PointCloud()
pcd.points = open3d.Vector3dVector(pts)
pcd.colors = open3d.Vector3dVector(colors / 255.0)

# for visualizing the origin
coord = open3d.create_mesh_coordinate_frame(1, [0, 0, 0])

# visualize point cloud
open3d.visualization.draw_geometries([pcd, coord])
