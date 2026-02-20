#!/usr/bin/env python3
from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import behav3d_commands


class MySession(behav3d_commands.Session):
    def run_scan_session(self, targets):
        self.run_sync(self.motion.home(enqueue=False))
        self.run_sync(self.motion.setSpd(0.1, enqueue=False))
        self.run_sync(self.motion.setAcc(0.1, enqueue=False))
        self.run_sync(self.motion.setEef("extruder_tcp", enqueue=False))
        self.run_sync(self.motion.setLIN(enqueue=False))
        for t in targets:
            plan_res = self.run_sync(
                self.motion.plan(x=t.x, y=t.y, z=t.z, enqueue=False)
            )
            if not plan_res.get("ok", False):
                self.node.get_logger().error("Plan failed; aborting scan session.")
                break
            self.run_sync(self.motion.exec(enqueue=False))
            self.run_sync(self.util.input(prompt="Press ENTER to start Capture!", enqueue=False))
            self.run_sync(
                self.camera.capture(
                    rgb=True,
                    depth=True,
                    ir=True,
                    folder="@session/scan_fib_simple",
                    enqueue=False,
                )
            )

    def run_disc_print_session(self, targets):
        self.run_sync(self.motion.home(enqueue=False))

        for i, t in enumerate(targets):
            plan_res = self.run_sync(
                self.motion.plan(x=t.x, y=t.y, z=t.z, enqueue=False)
            )
            if not plan_res.get("ok", False):
                raise ValueError(f"Failed to plan motion for target {t}")

            if i == 0:
                self.run_sync(self.motion.exec(enqueue=False))
            else:
                group = [
                    self.motion.exec(enqueue=False),
                    self.extruder.setExtruder(True, speed=600, enqueue=False)
                ]
                self.run_group(group)
                self.run_sync(self.util.wait(0.01, enqueue=False))
                self.run_sync(self.extruder.setExtruder(False, enqueue=False))

        # Barrier: wait for the last group to finish before returning.
        self.run_sync(self.util.wait(0.01, enqueue=False))

    def run_tsdf_cropped_reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        scan_folder: str = "manual_caps",
        visualize: bool = False,
        device: str = "CPU:0",
        timeout_s: Optional[float] = None,
        wait_for_outputs: bool = False,
        wait_timeout_s: float = 180.0,
    ):
        start_ts = time.time()
        res = self.run_sync(
            self.camera.reconstruct_tsdf_cropped(
                use_latest=use_latest,
                session_path=session_path,
                scan_folder=scan_folder,
                visualize=visualize,
                device=device,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not wait_for_outputs or not res.get("ok", False):
            return res

        metrics = res.get("metrics", {})
        output_path = str(metrics.get("mesh_path", "")).strip()
        if not output_path:
            output_path = str(metrics.get("output_path", "")).strip()
        if output_path:
            self._wait_for_fresh_file_output(
                output_path=output_path,
                start_ts=start_ts,
                timeout_s=float(wait_timeout_s),
            )
        return res

    def run_color_to_depth_reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        scan_folder: str = "manual_caps",
        visualize: bool = False,
        timeout_s: Optional[float] = None,
        wait_for_outputs: bool = False,
        wait_timeout_s: float = 30.0,
    ):
        start_ts = time.time()
        res = self.run_sync(
            self.camera.reconstruct_color_to_depth(
                use_latest=use_latest,
                session_path=session_path,
                scan_folder=scan_folder,
                visualize=visualize,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not wait_for_outputs or not res.get("ok", False):
            return res

        output_path = str(res.get("metrics", {}).get("output_path", ""))
        if output_path:
            self._wait_for_fresh_alignment_output(
                output_path=output_path,
                start_ts=start_ts,
                timeout_s=float(wait_timeout_s),
            )
        return res

    def run_color_to_depth_grid_sweep_reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        scan_folder: str = "grid_sweep",
        visualize: bool = False,
        timeout_s: Optional[float] = None,
        wait_for_outputs: bool = False,
        wait_timeout_s: float = 30.0,
    ):
        return self.run_color_to_depth_reconstruct(
            use_latest=use_latest,
            session_path=session_path,
            scan_folder=scan_folder,
            visualize=visualize,
            timeout_s=timeout_s,
            wait_for_outputs=wait_for_outputs,
            wait_timeout_s=wait_timeout_s,
        )

    def run_tsdf_grid_sweep_reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        scan_folder: str = "grid_sweep",
        visualize: bool = False,
        device: str = "CPU:0",
        timeout_s: Optional[float] = None,
        wait_for_outputs: bool = False,
        wait_timeout_s: float = 180.0,
    ):
        return self.run_tsdf_cropped_reconstruct(
            use_latest=use_latest,
            session_path=session_path,
            scan_folder=scan_folder,
            visualize=visualize,
            device=device,
            timeout_s=timeout_s,
            wait_for_outputs=wait_for_outputs,
            wait_timeout_s=wait_timeout_s,
        )

    def run_update_world_mesh(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        mesh_path: str = "",
        ply_path: str = "",
        prefer: str = "mesh",
        wait_timeout_s: float = 60.0,
        timeout_s: Optional[float] = None,
    ):
        return self.run_sync(
            self.camera.update_world_mesh(
                use_latest=use_latest,
                session_path=session_path,
                mesh_path=mesh_path,
                ply_path=ply_path,
                prefer=prefer,
                wait_timeout_s=wait_timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )

    def _wait_for_fresh_alignment_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ):
        out_dir = Path(output_path)

        def _fresh_alignment_exists() -> bool:
            if not out_dir.exists() or not out_dir.is_dir():
                return False
            for path in out_dir.glob("color_in_depth*.png"):
                try:
                    if path.stat().st_mtime >= (start_ts - 0.25):
                        return True
                except OSError:
                    continue
            return False

        return self.run_sync(
            self.util.wait_until(
                predicate=_fresh_alignment_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )

    def _wait_for_fresh_file_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ):
        out_path = Path(output_path)

        def _fresh_file_exists() -> bool:
            try:
                if not out_path.exists() or not out_path.is_file():
                    return False
                st = out_path.stat()
                if st.st_size <= 0:
                    return False
                return st.st_mtime >= (start_ts - 0.25)
            except OSError:
                return False

        return self.run_sync(
            self.util.wait_until(
                predicate=_fresh_file_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )
