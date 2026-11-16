"""Exercise service dispatch and completion handoff without requiring a ROS graph."""
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from behav3d_sense.reconstruct.utils import rectification


def completion_checker():
    root=Path(__file__).resolve().parents[2]
    path=root/'behav3d_orchestrator/behav3d_orchestrator/src/reconstruction_outputs.py'
    spec=importlib.util.spec_from_file_location('completion_check',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.alignment_is_complete


@pytest.fixture
def capture(tmp_path):
    scan=tmp_path/'scan';scan.mkdir()
    (scan/'manifest.yaml').write_text('captures: [{}, {}]')
    for name in ('depth','color','extrinsics'):
        (tmp_path/name).write_text(name)
    session=SimpleNamespace(path=tmp_path,_scan_folder='scan',depth_intrinsics_path=tmp_path/'depth',color_intrinsics_path=tmp_path/'color',_camera_extrinsics_path=tmp_path/'extrinsics')
    return session


@pytest.mark.parametrize('rectified',[False,True])
def test_wait_requires_complete_current_run_and_unchanged_images(capture,rectified):
    out=capture.path/'scan/reconstruct'/('color_in_depth_rectified' if rectified else 'color_in_depth')
    start=time.time();options=dict(rectified=rectified,run_id='new-run',started_at=start)
    check=completion_checker()
    rectification.write_alignment_metadata(out,capture,2,complete=False,**options)
    cv2.imwrite(str(out/'color_in_depth_0000.png'),np.zeros((8,8,3),np.uint8))
    assert not check(out,start)  # Regression: one fresh image must not start TSDF.
    with pytest.raises(ValueError):
        rectification.write_alignment_metadata(out,capture,2,complete=True,**options)
    cv2.imwrite(str(out/'color_in_depth_0001.png'),np.zeros((8,8,3),np.uint8))
    assert not check(out,start)  # Files alone are insufficient.
    rectification.write_alignment_metadata(out,capture,2,complete=True,**options)
    assert check(out,start)
    assert not check(out,start+1)  # An earlier successful run is not fresh.
    rectification.validate_alignment_metadata(out,capture,2,rectified=rectified)
    cv2.imwrite(str(out/'color_in_depth_0001.png'),np.ones((8,8,3),np.uint8))
    assert not check(out,start)
    with pytest.raises(ValueError,match='changed'):
        rectification.validate_alignment_metadata(out,capture,2,rectified=rectified)


def test_input_change_during_alignment_does_not_publish_success(capture):
    out=capture.path/'scan/reconstruct/color_in_depth_rectified'
    sig=rectification.alignment_signature(capture)
    rectification.write_alignment_metadata(out,capture,2,complete=False,input_signature=sig)
    Path(capture.depth_intrinsics_path).write_text('new calibration')
    with pytest.raises(ValueError,match='changed during'):
        rectification.write_alignment_metadata(out,capture,2,complete=True,input_signature=sig)
    assert not json.loads((out/rectification.ALIGNMENT_METADATA).read_text())['complete']


def test_old_rectified_record_is_readable_offline_but_not_a_fresh_ros_handoff(capture):
    out=capture.path/'scan/reconstruct/color_in_depth_rectified'
    out.mkdir(parents=True)
    old=dict(schema=1,rectified=True,complete=True,frame_count=2,
             input_signature=rectification.alignment_signature(capture))
    (out/rectification.LEGACY_RECTIFIED_METADATA).write_text(json.dumps(old))
    for i in range(2):
        cv2.imwrite(str(out/f'color_in_depth_{i:04d}.png'),np.zeros((8,8,3),np.uint8))
    rectification.validate_alignment_metadata(out,capture,2,rectified=True)
    assert len(rectification.aligned_image_paths(out,2))==2
    assert not completion_checker()(out,0)
    with pytest.raises(ValueError,match='different rectification'):
        rectification.validate_alignment_metadata(out,capture,2,rectified=False)


@pytest.fixture
def services(monkeypatch):
    class Node:
        def __init__(self,*args): self.params={}
        def declare_parameter(self,name,value):
            self.params[name]=SimpleNamespace(value=value)
            return self.params[name]
        def get_parameter(self,name): return self.params[name]
        def create_service(self,*args): return None
        def get_logger(self): return Mock()
    monkeypatch.setitem(sys.modules,'rclpy',SimpleNamespace())
    monkeypatch.setitem(sys.modules,'rclpy.node',SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules,'behav3d_interfaces.srv',SimpleNamespace(ColorToDepth=object,TsdfCropped=object,TsdfObjectExtract=object))
    path=Path(__file__).parents[1]/'behav3d_sense/reconstruct/reconstruct_services.py'
    spec=importlib.util.spec_from_file_location('behav3d_sense.reconstruct._service_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('enabled',[True,False])
def test_service_dispatch_keeps_alignment_and_tsdf_camera_models_together(services,monkeypatch,tmp_path,enabled):
    request=SimpleNamespace(session_path=str(tmp_path),use_latest=False,scan_folder='scan',visualize=False,device='')
    for cls in (services.ColorToDepthService,services.TSDFCroppedService,services.TSDFObjectExtractService):
        node=cls()
        assert node.get_parameter('rectify_images').value is True
        node.params['rectify_images'].value=enabled
        runner=Mock(return_value=None);node._runner=runner
        class ImmediateThread:
            def __init__(self,target,args,daemon): self.target=target;self.args=args
            def start(self): self.target(*self.args)
        monkeypatch.setattr(services.threading,'Thread',ImmediateThread)
        response=SimpleNamespace(success=False,message='',output_path='',mesh_path='',rgb_ply_path='',confidence_ply_path='')
        node.handle_request(request,response)
        assert response.success
        assert runner.call_args.kwargs['rectify_images'] is enabled
        if cls is services.ColorToDepthService:
            assert Path(response.output_path).name==('color_in_depth_rectified' if enabled else 'color_in_depth')


@pytest.mark.parametrize('enabled',[True,False])
def test_service_wrappers_forward_model_to_actual_reconstruction_modules(services,monkeypatch,tmp_path,enabled):
    for runner,module in ((services._run_color_to_depth,services.color_to_depth),
                          (services._run_tsdf_cropped,services.TSDF_cpu_cropped),
                          (services._run_tsdf_object_extract,services.TSDF_cpu_object_extract)):
        run=Mock();monkeypatch.setattr(module,'run',run)
        runner(tmp_path,'scan',False,rectify_images=enabled)
        assert run.call_args.kwargs['rectify_images'] is enabled
