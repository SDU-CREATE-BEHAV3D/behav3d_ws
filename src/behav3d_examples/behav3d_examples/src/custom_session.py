#!/usr/bin/env python3
from __future__ import annotations

import behav3d_commands


class MySession(behav3d_commands.Session):
    def run_scan_session(self, targets):
        self.run_sync(self.motion.home(enqueue=False))
        self.run_sync(self.motion.setSpd(0.2, enqueue=False))
        self.run_sync(self.motion.setAcc(0.2, enqueue=False))
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
                    self.extruder.print_steps(steps=2000, speed=500, enqueue=False),
                ]
                self.run_group(group)

        # Barrier: wait for the last group to finish before returning.
        self.run_sync(self.util.wait(0.01, enqueue=False))