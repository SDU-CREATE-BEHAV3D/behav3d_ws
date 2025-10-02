# file: behav3d_py/yaml_runner.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .commands import Commands
from .yaml_parser import SequenceParser

class RunYamlTest(Node):
    def __init__(self, yaml_path: str):
        super().__init__("run_yaml_test")
        self.cmd = Commands(self)
        self.parser = SequenceParser(self.cmd, logger=self.get_logger())
        self.parser.run_file(yaml_path)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", required=True, help="Path to YAML file")
    args = parser.parse_args()

    rclpy.init()
    node = RunYamlTest(args.path)
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
