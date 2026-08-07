from glob import glob
from setuptools import find_packages, setup


package_name = "semantic_exploration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tommaso De Pascale",
    maintainer_email="tomtom4830@gmail.com",
    description="Autonomous frontier exploration for semantic map construction.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "frontier_explorer = semantic_exploration.frontier_explorer:main",
            "navigation_posture = semantic_exploration.navigation_posture:main",
            "semantic_live_fallback = "
            "semantic_exploration.semantic_live_fallback:main",
            "save_mapping_state = semantic_exploration.save_mapping_state:main",
        ],
    },
)
