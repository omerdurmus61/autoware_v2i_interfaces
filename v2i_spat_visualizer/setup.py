from setuptools import find_packages, setup


package_name = "v2i_spat_visualizer"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/intersections.yaml"]),
        (f"share/{package_name}/launch", ["launch/v2i_spat_visualizer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="omer",
    maintainer_email="omercandurmuss@gmail.com",
    description="Qt-based SPaT visualizer for configured V2I signal groups.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spat_visualizer = v2i_spat_visualizer.spat_visualizer:main",
        ],
    },
)
