#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


class MeshUpdateHandler(FileSystemEventHandler):
    """Triggers callback when a mesh file appears or updates."""
    def __init__(self, callback, valid_exts=(".stl", ".obj", ".ply")):
        super().__init__()
        self.callback = callback
        self.valid_exts = valid_exts

    def on_modified(self, event):
        if not event.is_directory and event.src_path.endswith(self.valid_exts):
            self.callback(event.src_path)

    def on_created(self, event):
        self.on_modified(event)


class MeshVisualizer(Node):
    def __init__(self):
        super().__init__('mesh_visualizer')

        # === Default parameters (can still be overridden via launch) ===
        self.declare_parameter('mesh_dir', '/home/lab/robot/meshes')
        self.declare_parameter('frame_id', 'ur20_base_link')

        self.mesh_dir = self.get_parameter('mesh_dir').value
        self.frame_id = self.get_parameter('frame_id').value

        # === Publisher ===
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # === File watcher ===
        self.event_handler = MeshUpdateHandler(self.publish_latest_mesh)
        self.observer = Observer()
        try:
            if not os.path.exists(self.mesh_dir):
                os.makedirs(self.mesh_dir, exist_ok=True)
                self.get_logger().warn(f"Mesh directory not found — created: {self.mesh_dir}")
            self.observer.schedule(self.event_handler, self.mesh_dir, recursive=False)
            self.observer.start()
        except Exception as e:
            self.get_logger().error(f"Failed to start file watcher: {e}")

        self.get_logger().info("=== Mesh Visualizer Node Started ===")
        self.get_logger().info(f"📂 Watching directory: {self.mesh_dir}")
        self.get_logger().info(f"🗺️  Publishing in frame: {self.frame_id}")

        # === Wait for existing mesh (up to ~5 seconds) ===
        self.get_logger().info("Attempting initial mesh load...")
        for _ in range(10):  # 10 × 0.5s = 5 seconds total
            if any(f.lower().endswith(('.stl', '.obj', '.ply')) for f in os.listdir(self.mesh_dir)):
                self.publish_latest_mesh()
                break
            self.get_logger().info("⏳ No meshes yet — waiting...")
            time.sleep(0.5)
        else:
            self.get_logger().warn("⚠️ No mesh found after waiting; will publish on next update.")

    def publish_latest_mesh(self, file_path=None):
        """Publishes the newest mesh file as a visualization marker."""
        try:
            # Select the most recent valid mesh if none specified
            if file_path is None:
                meshes = [
                    os.path.join(self.mesh_dir, f)
                    for f in os.listdir(self.mesh_dir)
                    if f.lower().endswith(('.stl', '.obj', '.ply'))
                ]
                if not meshes:
                    self.get_logger().warn("No mesh files found in directory.")
                    return
                file_path = max(meshes, key=os.path.getmtime)

            abs_path = os.path.abspath(file_path)
            if not os.path.isfile(abs_path):
                self.get_logger().error(f"Mesh file not found: {abs_path}")
                return

            marker = Marker()
            marker.header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())
            marker.ns = 'reconstructed_mesh'
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = f"file:///{abs_path}"
            marker.pose = Pose()
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r = marker.color.g = marker.color.b = 0.8
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            self.get_logger().info(f"✅ Published mesh: {abs_path}")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish mesh: {e}")

    def destroy_node(self):
        try:
            self.observer.stop()
            self.observer.join()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MeshVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
