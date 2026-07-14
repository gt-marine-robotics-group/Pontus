"""Shared pytest configuration for the V2 YOLO/SONOPTIX fusion tests."""

from __future__ import annotations

import importlib
import os
from pathlib import Path
import sys
from typing import Any

import pytest
import yaml


_REQUIRED_TOPIC_KEYS = (
    "cameras",
    "camera_frame_template",
    "camera_info_topic_template",
    "detections_topic_template",
    "semantic_object_service",
    "pointcloud_cluster_topic",
)


def pytest_addoption(parser: pytest.Parser) -> None:
    """Add fusion-test command-line options."""
    parser.addoption(
        "--fusion-module",
        action="store",
        default=os.environ.get(
            "PONTUS_FUSION_MODULE",
            "pontus_mapping.cluster_coord",
        ),
        help=(
            "Installed Python module containing the fusion node. "
            "PONTUS_FUSION_MODULE provides the same setting."
        ),
    )
    parser.addoption(
        "--topics-yaml",
        action="store",
        default=os.environ.get("PONTUS_TOPICS_YAML"),
        help=(
            "Path to the production topics.yaml file. "
            "PONTUS_TOPICS_YAML provides the same setting."
        ),
    )


def _package_root() -> Path:
    """Return the pontus_mapping package root containing the test folder."""
    return Path(__file__).resolve().parents[1]


def _add_workspace_python_packages() -> None:
    """Make sibling source packages importable when testing before install."""
    search_root = _package_root().parent
    candidates = [
        path
        for path in search_root.rglob("pontus_bringup/topic_config.py")
        if not {"build", "install", "log"}.intersection(path.parts)
    ]

    for topic_config_path in candidates:
        source_package_root = topic_config_path.parent.parent
        source_path = str(source_package_root)
        if source_path not in sys.path:
            sys.path.insert(0, source_path)


def _read_topics_parameters(path: Path) -> dict[str, Any]:
    """Read and validate the wildcard ROS parameters in topics.yaml."""
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise pytest.UsageError(
            f"Could not read topics configuration '{path}': {error}"
        ) from error

    if not isinstance(document, dict):
        raise pytest.UsageError(
            f"Topics configuration must contain a YAML mapping: {path}"
        )

    wildcard_node = document.get("/**")
    if not isinstance(wildcard_node, dict):
        raise pytest.UsageError(
            f"Topics configuration is missing the '/**' section: {path}"
        )

    parameters = wildcard_node.get("ros__parameters")
    if not isinstance(parameters, dict):
        raise pytest.UsageError(
            f"Topics configuration is missing '/**/ros__parameters': {path}"
        )

    missing = [key for key in _REQUIRED_TOPIC_KEYS if key not in parameters]
    if missing:
        raise pytest.UsageError(
            f"Topics configuration '{path}' is missing required keys: {missing}"
        )

    cameras = parameters["cameras"]
    if not isinstance(cameras, list) or not cameras:
        raise pytest.UsageError(
            f"The 'cameras' parameter must be a nonempty list in: {path}"
        )

    return parameters


def _is_valid_topics_yaml(path: Path) -> bool:
    """Return whether a YAML file satisfies the fusion-node config contract."""
    try:
        _read_topics_parameters(path)
    except pytest.UsageError:
        return False
    return True


def _discover_topics_yaml() -> Path:
    """Locate the production topics.yaml near the pontus_mapping package."""
    package_root = _package_root()
    search_root = package_root.parent

    preferred_paths = (
        search_root / "pontus_bringup" / "config" / "topics.yaml",
        package_root / "config" / "topics.yaml",
    )
    for path in preferred_paths:
        if path.is_file() and _is_valid_topics_yaml(path):
            return path.resolve()

    candidates = [
        path.resolve()
        for path in search_root.rglob("topics.yaml")
        if not {"build", "install", "log"}.intersection(path.parts)
        and _is_valid_topics_yaml(path)
    ]
    candidates = sorted(set(candidates))

    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise pytest.UsageError(
            "Could not locate a topics.yaml containing the fusion-node "
            "parameters. Set PONTUS_TOPICS_YAML or pass --topics-yaml."
        )

    formatted = "\n".join(f"  - {path}" for path in candidates)
    raise pytest.UsageError(
        "Found multiple valid topics.yaml files. Set PONTUS_TOPICS_YAML or "
        f"pass --topics-yaml. Candidates:\n{formatted}"
    )


@pytest.fixture(scope="session")
def fusion_module(pytestconfig: pytest.Config):
    """Import the production fusion module under test."""
    _add_workspace_python_packages()
    module_name = pytestconfig.getoption("--fusion-module")
    try:
        return importlib.import_module(module_name)
    except ModuleNotFoundError as error:
        raise pytest.UsageError(
            f"Could not import fusion module '{module_name}': {error}. "
            "Build and source the workspace, or set PONTUS_FUSION_MODULE."
        ) from error


@pytest.fixture(scope="session")
def topics_yaml_path(pytestconfig: pytest.Config) -> Path:
    """Return the production topics.yaml path used by TopicConfig."""
    configured_path = pytestconfig.getoption("--topics-yaml")
    if configured_path:
        path = Path(configured_path).expanduser().resolve()
        if not path.is_file():
            raise pytest.UsageError(
                f"Configured topics.yaml does not exist: {path}"
            )
        _read_topics_parameters(path)
        return path

    return _discover_topics_yaml()


@pytest.fixture(scope="session")
def topics_config(topics_yaml_path: Path) -> dict[str, Any]:
    """Return the parameters from the same YAML consumed by TopicConfig."""
    return _read_topics_parameters(topics_yaml_path)
