import numpy as np
import open3d as o3d

# pcd = o3d.io.read_point_cloud(
#     "C:/Users/tcous/Desktop/school/CSCE748/project/StructureFromMotion/Code/results/castle-P30/point-clouds/cloud_30_view.ply", format="ply")
pcd = o3d.io.read_point_cloud(
    "./castle_nview.ply", format="ply")
o3d.visualization.draw_geometries([pcd])
