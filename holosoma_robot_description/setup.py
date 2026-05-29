from pathlib import Path

from setuptools import find_packages, setup

package_name = "holosoma_robot_description"
package_dir = Path(__file__).resolve().parent
urdf_dir = package_dir / "urdf"
mesh_dir = urdf_dir / "meshes"


def data_file_path(path: Path) -> str:
    return path.relative_to(package_dir).as_posix()


mesh_files = [data_file_path(path) for path in sorted(mesh_dir.iterdir()) if path.is_file()] if mesh_dir.is_dir() else []

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/g1_robot_state_publisher.launch.py"]),
        (f"share/{package_name}/urdf", [data_file_path(urdf_dir / "g1_29dof.urdf")]),
        (f"share/{package_name}/urdf/meshes", mesh_files),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "initial_map_frame_bootstrap = holosoma_robot_description.initial_map_frame_bootstrap:main",
            "robot_state_to_joint_state = holosoma_robot_description.robot_state_to_joint_state:main",
            "unitree_lowstate_to_joint_state = holosoma_robot_description.unitree_lowstate_to_joint_state:main",
        ],
    },
    zip_safe=True,
    maintainer="phc",
    maintainer_email="phc@example.com",
    description="Robot description and launch files for Holosoma G1 ROS2 TF publishing.",
    license="Apache-2.0",
)
