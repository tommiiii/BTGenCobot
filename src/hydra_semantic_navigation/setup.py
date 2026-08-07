import os
from glob import glob

from setuptools import find_packages, setup


package_name = "hydra_semantic_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BTGenCobot Team",
    maintainer_email="noreply@example.com",
    description="Hydra DSG grounding and topological navigation adapter",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scene_graph_adapter = hydra_semantic_navigation.scene_graph_adapter:main",
        ],
    },
)
