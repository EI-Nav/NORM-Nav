#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS I/O helpers for object modeling node.

Provides small abstractions for publishing cached outputs so unit tests
can bypass ROS publishers more easily.
"""

from typing import Any


class PublisherCache:
    """Publish cached artifacts stored on the node."""

    def __init__(self, node: Any) -> None:
        self.node = node

    def publish(self) -> None:
        """Publish cached point cloud, markers, and OBB info if available."""
        # Point cloud
        if getattr(self.node, "latest_modeled_pc", None) is not None:
            self.node.modeled_pc_pub.publish(self.node.latest_modeled_pc)

        # Markers
        if (
            getattr(self.node, "publish_obb_markers", False)
            and getattr(self.node, "latest_marker_array", None) is not None
            and len(self.node.latest_marker_array.markers) > 0
        ):
            self.node.object_model_marker_pub.publish(self.node.latest_marker_array)

        # OBB info
        if getattr(self.node, "publish_obb_info", False) and getattr(self.node, "latest_obb_info_array", None) is not None:
            self.node.obb_info_pub.publish(self.node.latest_obb_info_array)


class MarkerBuilder:
    """Placeholder for future marker construction utilities."""

    pass


class ObbInfoBuilder:
    """Placeholder for future OBB info construction utilities."""

    pass


