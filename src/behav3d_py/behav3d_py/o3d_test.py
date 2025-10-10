#!/usr/bin/env python3
import os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
import open3d as o3d

class O3dTestNode(Node):
    def __init__(self):
        super().__init__("o3d_test")
        self.declare_parameter("visualize", False)
        self.declare_parameter("outdir", str(Path.home() / "o3d_smoke_out"))
        visualize = self.get_parameter("visualize").get_parameter_value().bool_value
        outdir = Path(self.get_parameter("outdir").get_parameter_value().string_value)
        outdir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Open3D version: {o3d.__version__}")
        # demo geom
        pts = np.array([[0,0,0],[0.1,0,0],[0,0.1,0],[0,0,0.1]], dtype=np.float64)
        colors = np.array([[1,0,0],[0,1,0],[0,0,1],[1,1,1]], dtype=np.float64)
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        pcd.colors = o3d.utility.Vector3dVector(colors)
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        o3d.io.write_point_cloud(str(outdir / "smoke_points.ply"), pcd)
        o3d.io.write_triangle_mesh(str(outdir / "smoke_axes.ply"), mesh)
        if visualize and os.environ.get("DISPLAY"):
            o3d.visualization.draw_geometries([pcd, mesh])
        rclpy.shutdown()

def main():
    rclpy.init()
    node = O3dTestNode()
    rclpy.spin_once(node)
