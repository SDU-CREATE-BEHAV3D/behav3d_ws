#!/usr/bin/env python3
"""Helpers for synchronously updating parameters on another ROS node."""

from __future__ import annotations

import time
from typing import Sequence

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


def set_remote_parameters(
    node,
    *,
    remote_node_name: str,
    params: Sequence[Parameter],
    timeout_s: float,
    label: str,
) -> str:
    node_name = str(remote_node_name or "/behav3d_fields").strip()
    if not node_name.startswith("/"):
        node_name = f"/{node_name}"
    timeout = max(0.1, float(timeout_s))
    client = AsyncParameterClient(node, node_name)
    if hasattr(client, "wait_for_services"):
        ready = bool(client.wait_for_services(timeout_sec=timeout))
    elif hasattr(client, "wait_for_service"):
        ready = bool(client.wait_for_service(timeout_sec=timeout))
    else:
        ready = bool(
            client.services_are_ready()
            if hasattr(client, "services_are_ready")
            else False
        )
    if not ready:
        raise RuntimeError(f"Parameter service for {node_name} not available.")

    future = client.set_parameters(list(params))
    deadline = time.time() + timeout
    while rclpy.ok() and not future.done() and time.time() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise TimeoutError(f"Timed out while setting {label} params on {node_name}.")

    response = future.result()
    if response is None:
        raise RuntimeError(f"Failed to set {label} params on {node_name}: no response.")
    if hasattr(response, "results"):
        results = list(response.results)
    elif isinstance(response, (list, tuple)):
        results = list(response)
    else:
        raise RuntimeError(
            f"Failed to set {label} params: unexpected response type "
            f"{type(response).__name__}"
        )
    failed = [str(result.reason) for result in results if not result.successful]
    if failed:
        raise RuntimeError(f"Failed to set {label} params: {'; '.join(failed)}")
    return node_name
