"""Static safeguards for the review-only Docker image and Compose service."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]
PACKAGES = {
    'behav3d_interfaces', 'behav3d_utils',
    'behav3d_commands', 'behav3d_orchestrator',
}


class DockerReviewConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compose = yaml.safe_load((ROOT / 'compose.ros-review.yaml').read_text())
        cls.service = cls.compose['services']['ros-review']
        cls.dockerfile = (ROOT / 'docker/ros-review/Dockerfile').read_text()
        cls.validation = (ROOT / 'docker/ros-review/validate.sh').read_text()

    def test_only_review_service_and_curated_build(self):
        self.assertEqual(set(self.compose['services']), {'ros-review'})
        self.assertEqual(self.service['build']['context'], '.')
        self.assertEqual(self.service['build']['dockerfile'], 'docker/ros-review/Dockerfile')
        self.assertEqual(self.service['command'], ['/opt/behav3d_review/validate.sh'])
        self.assertIn('linux/arm64', self.service['platform'])

    def test_runtime_has_no_external_network_or_device_privileges(self):
        self.assertEqual(self.service['network_mode'], 'none')
        self.assertEqual(self.service['cap_drop'], ['ALL'])
        self.assertIn('no-new-privileges:true', self.service['security_opt'])
        for key in ('devices', 'device_cgroup_rules', 'ports', 'cap_add', 'network',
                    'pid', 'ipc', 'volumes_from', 'external_links'):
            self.assertNotIn(key, self.service)
        self.assertFalse(self.service.get('privileged', False))
        self.assertNotIn('docker.sock', yaml.safe_dump(self.compose))

    def test_nonroot_read_only_runtime_with_bounded_resources(self):
        self.assertEqual(self.service['user'], '10001:10001')
        self.assertIs(self.service['read_only'], True)
        self.assertEqual(self.service['pids_limit'], 256)
        self.assertEqual(self.service['mem_limit'], '4g')
        self.assertIn('USER 10001:10001', self.dockerfile)
        self.assertEqual(len(self.service['tmpfs']), 1)
        self.assertTrue(self.service['tmpfs'][0].startswith('/tmp:rw,nosuid,nodev,size='))

    def test_only_checkout_and_results_are_mounted_by_default(self):
        volumes = self.service['volumes']
        self.assertEqual(len(volumes), 2)
        source = next(v for v in volumes if v['type'] == 'bind')
        self.assertEqual(source['source'], '.')
        self.assertEqual(source['target'], '/workspace/behav3d_ws')
        self.assertIs(source['read_only'], True)
        self.assertIs(source['bind']['create_host_path'], False)
        result = next(v for v in volumes if v['type'] == 'volume')
        self.assertEqual(result['target'], '/results')
        self.assertEqual(set(self.compose['volumes']), {result['source']})

    def test_evidence_override_is_explicit_read_only_and_preserves_paths(self):
        override = yaml.safe_load((ROOT / 'compose.ros-review.evidence.yaml').read_text())
        self.assertEqual(set(override), {'services'})
        self.assertEqual(set(override['services']), {'ros-review'})
        service = override['services']['ros-review']
        self.assertEqual(set(service), {'volumes'})
        self.assertEqual(len(service['volumes']), 3)
        for volume in service['volumes']:
            self.assertEqual(volume['type'], 'bind')
            self.assertEqual(volume['source'], volume['target'])
            self.assertIn(':?', volume['source'])
            self.assertIs(volume['read_only'], True)
            self.assertIs(volume['bind']['create_host_path'], False)
        self.assertTrue(service['volumes'][0]['source'].endswith('/outputs'))
        self.assertTrue(service['volumes'][1]['source'].endswith('/outputs'))

    def test_ros_environment_and_middleware_are_loopback_only(self):
        self.assertIn('chmod -R a+rX /opt/behav3d_ws/install', self.dockerfile)
        self.assertIn('ros2 pkg prefix "${review_package}"', self.validation)
        env = self.service['environment']
        self.assertEqual(env['BEHAV3D_REVIEW_CONTAINER'], '1')
        self.assertEqual(env['ROS_DOMAIN_ID'], '231')
        self.assertEqual(env['ROS_LOCALHOST_ONLY'], '1')
        self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_cyclonedds_cpp')
        tree = ET.parse(ROOT / 'docker/ros-review/cyclonedds.xml')
        ns = {'c': 'https://cdds.io/config'}
        interfaces = tree.findall('.//c:NetworkInterface', ns)
        self.assertEqual([item.attrib['name'] for item in interfaces], ['lo'])
        self.assertEqual(tree.findtext('.//c:AllowMulticast', namespaces=ns), 'false')
        peers = tree.findall('.//c:Peer', ns)
        self.assertEqual([item.attrib['Address'] for item in peers], ['127.0.0.1'])

    def test_docker_context_is_an_allowlist_without_recordings_or_secrets(self):
        lines = [line.strip() for line in (ROOT / '.dockerignore').read_text().splitlines()
                 if line.strip() and not line.startswith('#')]
        self.assertEqual(lines[0], '**')
        expected = {'!src/', '!docker/', '!docker/ros-review/', '!docker/ros-review/**'}
        # The optional startup image uses source-only numerical kernels, not a
        # fifth ROS package, recordings, scalar demos or legacy robot nodes.
        expected.update({'!src/behav3d_py/', '!src/behav3d_py/behav3d_py/',
                         '!src/behav3d_py/behav3d_py/scalar_field/',
                         '!src/behav3d_py/behav3d_py/scalar_field/lib_scalar/',
                         '!src/behav3d_py/behav3d_py/scalar_field/lib_scalar/*.py'})
        for package in PACKAGES:
            expected.update({'!src/' + package + '/', '!src/' + package + '/**'})
        self.assertEqual({line for line in lines if line.startswith('!')}, expected)
        for pattern in ('**/.git', '**/.venv', '**/.env', '**/.env.*',
                        '**/captures', '**/outputs', '**/build', '**/install', '**/log'):
            self.assertIn(pattern, lines)

    def test_dockerfile_copies_and_builds_only_four_review_packages(self):
        copied = set(re.findall(r'^COPY src/([^/]+)/', self.dockerfile, re.MULTILINE))
        self.assertEqual(copied, PACKAGES)
        joined = self.dockerfile.replace('\\\n', ' ')
        match = re.search(r'--packages-select\s+(.+?)\s+--event-handlers', joined)
        self.assertIsNotNone(match)
        self.assertEqual(set(match.group(1).split()), PACKAGES)
        self.assertIn('ARG ROS_IMAGE=ros:jazzy-ros-base', self.dockerfile)
        self.assertIn('FROM ${ROS_IMAGE}', self.dockerfile)
        self.assertIn('-DBUILD_TESTING=OFF', self.dockerfile)
        self.assertNotIn('rosdep install', joined)
        for package in ('ros-jazzy-ur ', 'ros-jazzy-moveit ', 'ros-jazzy-gazebo',
                        'pymodbus', 'orbbec_camera'):
            self.assertNotIn(package, self.dockerfile)

    def test_runner_checks_source_and_keeps_real_ros_separate_from_mocks(self):
        self.assertIn('set -euo pipefail', self.validation)
        self.assertIn('sha256sum --check --status', self.validation)
        self.assertIn('source-sha256.txt', self.validation)
        self.assertIn('mktemp -d /results/review-validation-', self.validation)
        self.assertIn('test_docker_review_config.py', self.validation)
        self.assertIn('python3 -B "${review_repo}/scripts/ros_review_smoke.py"', self.validation)
        self.assertNotIn('print_move.launch.py', self.validation)
        self.assertNotIn('--offline-demonstration-approval', self.validation)

    def test_entrypoint_refuses_root_or_nonisolated_environment(self):
        entrypoint = (ROOT / 'docker/ros-review/entrypoint.sh').read_text()
        self.assertIn('BEHAV3D_REVIEW_CONTAINER', entrypoint)
        self.assertIn('ROS_LOCALHOST_ONLY', entrypoint)
        self.assertIn('ROS_DOMAIN_ID', entrypoint)
        self.assertIn('rmw_cyclonedds_cpp', entrypoint)
        self.assertIn('/.dockerenv', entrypoint)
        self.assertIn('$(id -u)', entrypoint)
        self.assertIn('exit 64', entrypoint)
        self.assertIn('exec "$@"', entrypoint)


if __name__ == '__main__':
    unittest.main()
