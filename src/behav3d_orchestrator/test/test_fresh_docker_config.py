"""Static safeguards for optional fresh-analysis Docker validation tooling."""
from pathlib import Path
import re
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[3]
ANALYSIS_PYTHON = '/opt/behav3d_analysis/bin/python'


class FreshDockerConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.override = yaml.safe_load((ROOT / 'compose.ros-review.fresh.yaml').read_text())
        cls.service = cls.override['services']['ros-review']
        cls.base = yaml.safe_load((ROOT / 'compose.ros-review.yaml').read_text())['services']['ros-review']
        cls.dockerfile = (ROOT / 'docker/ros-review/Dockerfile.fresh').read_text()

    def test_optional_amd64_image_extends_the_review_base(self):
        self.assertEqual(set(self.override), {'services'})
        self.assertEqual(set(self.override['services']), {'ros-review'})
        self.assertEqual(self.service['image'], 'behav3d-ros-fresh:jazzy-amd64')
        self.assertEqual(self.service['platform'], 'linux/amd64')
        self.assertEqual(self.service['build']['context'], '.')
        self.assertEqual(self.service['build']['dockerfile'], 'docker/ros-review/Dockerfile.fresh')
        self.assertIn('behav3d-ros-review:jazzy-amd64', self.service['build']['args']['REVIEW_IMAGE'])
        self.assertIn('ARG REVIEW_IMAGE=behav3d-ros-review:jazzy-amd64', self.dockerfile)
        self.assertIn('FROM ${REVIEW_IMAGE}', self.dockerfile)
        self.assertIn('test "$(dpkg --print-architecture)" = amd64', self.dockerfile)

    def test_runtime_preserves_review_isolation(self):
        self.assertEqual(self.service['network_mode'], 'none')
        self.assertIs(self.service['read_only'], True)
        self.assertEqual(self.service['user'], '10001:10001')
        self.assertEqual(self.service['cap_drop'], ['ALL'])
        self.assertEqual(self.service['security_opt'], ['no-new-privileges:true'])
        self.assertEqual(self.base['pids_limit'], 256)
        self.assertEqual(self.base['mem_limit'], '4g')
        for key in ('entrypoint', 'command', 'environment', 'devices', 'device_cgroup_rules',
                    'ports', 'cap_add', 'network', 'pid', 'ipc', 'volumes_from', 'external_links'):
            self.assertNotIn(key, self.service)
        self.assertFalse(self.service.get('privileged', False))
        self.assertNotIn('docker.sock', yaml.safe_dump(self.override))

    def test_dependencies_are_pinned_binary_wheels_in_an_isolated_venv(self):
        self.assertIn('python3 -m venv --system-site-packages /opt/behav3d_analysis', self.dockerfile)
        self.assertIn(ANALYSIS_PYTHON + ' -m pip install', self.dockerfile)
        self.assertIn('--no-cache-dir --only-binary=:all:', self.dockerfile)
        pins = {
            'numpy': '1.26.4', 'open3d-cpu': '0.19.0',
            'opencv-python-headless': '4.10.0.84', 'scikit-image': '0.23.2',
            'tyro': '1.0.13', 'plotly': '6.8.0',
        }
        self.assertEqual(dict(re.findall(r'\b([a-z][a-z0-9-]*)==([0-9][0-9.]*)', self.dockerfile)), pins)
        self.assertIn('python3-venv libegl1 libgl1 libgomp1', self.dockerfile)
        self.assertNotIn('--break-system-packages', self.dockerfile)
        self.assertNotIn('ENV PATH=', self.dockerfile)
        self.assertNotRegex(self.dockerfile, r'\bpython3 -m pip install\b')
        self.assertTrue(self.dockerfile.rstrip().endswith('USER 10001:10001'))

    def test_build_smokes_cpu_imports_and_records_the_environment(self):
        self.assertIn('import cv2, numpy, open3d, plotly, scipy, skimage, tyro', self.dockerfile)
        self.assertIn('from PIL import Image', self.dockerfile)
        self.assertIn('open3d.core.Device("CPU:0")', self.dockerfile)
        self.assertIn('open3d.__version__ == "0.19.0"', self.dockerfile)
        self.assertIn('/opt/behav3d_ws/fresh-python-packages.txt', self.dockerfile)
        self.assertIn('/opt/behav3d_ws/fresh-apt-packages.txt', self.dockerfile)
        for name in ('pymodbus', 'orbbec_camera', 'ros-jazzy-ur', 'ros-jazzy-moveit ',
                     'torch==', 'transformers==', 'litellm=='):
            self.assertNotIn(name, self.dockerfile)

    def test_only_required_external_code_and_recording_are_mounted(self):
        volumes = self.service['volumes']
        self.assertEqual(len(volumes), 5)
        roots = {
            'BEHAV3D_AGENT007_ROOT': {'/src', '/scripts'},
            'BEHAV3D_DDS_ROOT': {'/src', '/examples'},
            'BEHAV3D_RECORDING_ROOT': {''},
        }
        seen = {key: set() for key in roots}
        for volume in volumes:
            self.assertEqual(volume['type'], 'bind')
            self.assertEqual(volume['source'], volume['target'])
            self.assertIs(volume['read_only'], True)
            self.assertIs(volume['bind']['create_host_path'], False)
            match = re.fullmatch(r'\$\{([A-Z0-9_]+):\?[^}]+\}(.*)', volume['source'])
            self.assertIsNotNone(match)
            variable, suffix = match.groups()
            self.assertIn(variable, roots)
            self.assertIn(suffix, roots[variable])
            seen[variable].add(suffix)
        self.assertEqual(seen, roots)
        rendered = yaml.safe_dump(self.override)
        for forbidden in ('/.venv', '/venv', '/.git', '/.env', '/outputs', '/var/run', '/Users/'):
            self.assertNotIn(forbidden, rendered)

    def test_base_checkout_and_results_mounts_remain_unchanged(self):
        self.assertNotIn('volumes', self.override)
        checkout = next(volume for volume in self.base['volumes'] if volume['type'] == 'bind')
        self.assertEqual(checkout['source'], '.')
        self.assertEqual(checkout['target'], '/workspace/behav3d_ws')
        self.assertIs(checkout['read_only'], True)
        results = next(volume for volume in self.base['volumes'] if volume['type'] == 'volume')
        self.assertEqual(results['target'], '/results')
        self.assertNotIn('/results', {volume['target'] for volume in self.service['volumes']})


if __name__ == '__main__':
    unittest.main()
