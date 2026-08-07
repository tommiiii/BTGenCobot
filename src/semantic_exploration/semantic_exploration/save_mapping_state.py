"""Save the SLAM map and official Hydra DSG as one user operation."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tempfile

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from std_srvs.srv import Trigger


class MappingStateSaver(Node):
    def __init__(self, map_name: str) -> None:
        super().__init__("save_mapping_state")
        self._map_name = map_name
        self._map_client = self.create_client(
            SaveMap, "/slam_toolbox/save_map"
        )
        self._hydra_prepare_client = self.create_client(
            Trigger, "/hydra/prepare_scene_graph_save"
        )
        self._hydra_commit_client = self.create_client(
            Trigger, "/hydra/commit_scene_graph_save"
        )
        self._hydra_abort_client = self.create_client(
            Trigger, "/hydra/abort_scene_graph_save"
        )
        self._hydra_validation_client = self.create_client(
            Trigger, "/hydra/validate_scene_graph"
        )

    def save(self) -> bool:
        if not self._map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("SLAM map-save service is unavailable")
            return False
        transaction_clients = (
            self._hydra_prepare_client,
            self._hydra_commit_client,
            self._hydra_abort_client,
        )
        if not all(
            client.wait_for_service(timeout_sec=10.0)
            for client in transaction_clients
        ):
            self.get_logger().error(
                "Hydra transactional save services are unavailable"
            )
            return False
        if self._hydra_validation_client.wait_for_service(timeout_sec=2.0):
            validation_future = self._hydra_validation_client.call_async(
                Trigger.Request()
            )
            rclpy.spin_until_future_complete(
                self, validation_future, timeout_sec=30.0
            )
            validation = validation_future.result()
            if validation is None:
                self.get_logger().warning(
                    "Graph quality check did not respond; saving anyway"
                )
            elif validation.success:
                self.get_logger().info(validation.message)
            else:
                self.get_logger().warning(
                    f"Graph quality check: {validation.message}; saving anyway"
                )
        else:
            self.get_logger().warning(
                "Graph quality service is unavailable; saving anyway"
            )

        prepare_future = self._hydra_prepare_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, prepare_future, timeout_sec=60.0)
        prepared = prepare_future.result()
        if prepared is None or not prepared.success:
            message = prepared.message if prepared else "no response"
            self.get_logger().error(f"Hydra prepare-save failed: {message}")
            return False
        if "quality warning:" in prepared.message:
            self.get_logger().warning(prepared.message)
        else:
            self.get_logger().info(prepared.message)

        map_base = Path(self._map_name)
        map_paths = (
            map_base.with_suffix(".yaml"),
            map_base.with_suffix(".pgm"),
        )
        with tempfile.TemporaryDirectory(prefix="btgencobot-map-save-") as temp:
            backup_dir = Path(temp)
            existing = {}
            for path in map_paths:
                if path.exists():
                    backup = backup_dir / path.name
                    shutil.copy2(path, backup)
                    existing[path] = backup

            map_request = SaveMap.Request()
            map_request.name.data = self._map_name
            map_future = self._map_client.call_async(map_request)
            rclpy.spin_until_future_complete(self, map_future, timeout_sec=60.0)
            map_response = map_future.result()
            if (
                map_response is None
                or map_response.result != SaveMap.Response.RESULT_SUCCESS
            ):
                self._abort_pending_hydra_save()
                self.get_logger().error("SLAM Toolbox did not save the map")
                return False

            commit_future = self._hydra_commit_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                self, commit_future, timeout_sec=60.0
            )
            committed = commit_future.result()
            if committed is None or not committed.success:
                for path in map_paths:
                    if path in existing:
                        shutil.copy2(existing[path], path)
                    else:
                        path.unlink(missing_ok=True)
                message = committed.message if committed else "no response"
                self.get_logger().error(
                    f"Hydra commit-save failed; restored previous map: {message}"
                )
                return False

        self.get_logger().info(
            f"Saved SLAM map as {self._map_name} and {committed.message}"
        )
        return True

    def _abort_pending_hydra_save(self) -> None:
        future = self._hydra_abort_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map-name",
        default="/workspace/maps/house_pick_and_place",
        help="SLAM Toolbox output basename (without .yaml/.pgm)",
    )
    parsed, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    node = MappingStateSaver(parsed.map_name)
    success = node.save()
    node.destroy_node()
    rclpy.shutdown()
    if not success:
        raise SystemExit(1)
